import json
import sys
import time
import os
from typing import Any

import numpy as np

# 新增：支持 ZMQ（必需）。如果未安装 zmq，直接报错退出。
try:
    import zmq  # type: ignore
except Exception:  # pragma: no cover
    zmq = None

# 通过环境变量控制调试输出，避免影响正常性能/日志量
_DEBUG = os.environ.get("QPOASES_IPC_DEBUG", "0").lower() in {"1", "true", "yes", "on"}


def _dlog(msg: str) -> None:
    """调试日志：只输出到 stderr，避免污染 stdout(RPC 通道)。"""
    if not _DEBUG:
        return
    sys.stderr.write(f"[qpoases-ipc][debug] {msg}\n")
    sys.stderr.flush()


def _readline() -> str:
    # 旧 stdin/stdout IPC 模式已废弃：不再读取 stdin。
    raise RuntimeError("stdio IPC mode has been removed; use ZMQ")


def _ok(**kwargs: Any) -> None:
    # 旧 stdin/stdout IPC 模式已废弃：不再使用 stdout JSONL。
    raise RuntimeError("stdio IPC mode has been removed; use ZMQ")


def _err(message: str) -> None:
    # 旧 stdin/stdout IPC 模式已废弃：不再使用 stdout JSONL。
    sys.stderr.write(f"[qpoases-zmq] {message}\n")
    sys.stderr.flush()


def _handle_request(req: dict, *, qp_state: dict) -> dict:
    """处理一次 QP 请求（与传输层无关）。"""
    t0 = time.perf_counter()
    try:
        QProblem = qp_state["QProblem"]
        options = qp_state["options"]
        qp = qp_state.get("qp")

        cmd = req.get("cmd")
        if cmd == "close":
            qp_state["qp"] = None
            _dlog("received close")
            return {"status": "ok"}

        # 允许无约束调用：客户端可以不提供 A/lbA/ubA。
        H = np.asarray(req["H"], dtype=float)
        g = np.asarray(req["g"], dtype=float).reshape(-1)

        A_raw = req.get("A", None)
        lbA_raw = req.get("lbA", None)
        ubA_raw = req.get("ubA", None)

        has_constraints = A_raw is not None and lbA_raw is not None and ubA_raw is not None
        if has_constraints:
            A = np.asarray(A_raw, dtype=float)
            lbA = np.asarray(lbA_raw, dtype=float).reshape(-1)
            ubA = np.asarray(ubA_raw, dtype=float).reshape(-1)
            nC = int(A.shape[0])
        else:
            A = None
            lbA = None
            ubA = None
            nC = 0

        # 关键：Python 接口传入的是“数学意义”的矩阵/向量，不要盲目 .T。
        # C++ 里转置通常是为了 Eigen 内存布局与 qpOASES 的列主序/指针接口适配。
        nV = int(H.shape[0])

        print(H[8:16, 8:16])
        print(g[8:16])
        # 与 C++ 示例一致：nWSR = 5*(nV+nC)
        nWSR = np.asarray([int(5 * (nV + nC))])

        # 额外检查：避免 silent wrong-shape 导致结果异常（例如全 0）
        if H.ndim != 2 or H.shape[0] != H.shape[1]:
            raise ValueError(f"H must be square 2D, got shape={H.shape}")
        if g.shape[0] != nV:
            raise ValueError(f"g length mismatch: len(g)={g.shape[0]} nV={nV}")
        if has_constraints:
            if A is None:
                raise ValueError("internal: A is None but has_constraints=True")
            if A.ndim != 2 or A.shape[1] != nV:
                raise ValueError(f"A must be 2D with shape (nC,nV), got {A.shape}, nV={nV}")
            if lbA.shape[0] != nC or ubA.shape[0] != nC:
                raise ValueError(f"lbA/ubA length mismatch: lbA={lbA.shape} ubA={ubA.shape} nC={nC}")

        _dlog(
            f"recv cmd={cmd} nV={nV} nC={nC} "
            f"H.shape={H.shape} g.shape={g.shape} "
            f"A.shape={(None if A is None else A.shape)} "
            f"||g||={float(np.linalg.norm(g)):.6g}"
        )

        if qp is None or cmd == "init":
            qp = QProblem(nV, nC)
            qp.setOptions(options)
            if has_constraints:
                qp.init(H, g, A, None, None, lbA, ubA, nWSR)
            else:
                qp.init(H, g, None, None, None, None, None, nWSR)
            qp_state["qp"] = qp
        else:
            if has_constraints:
                qp.hotstart(H, g, A, None, None, lbA, ubA, nWSR)
            else:
                qp.hotstart(H, g, None, None, None, None, None, nWSR)

        x = np.ones(nV)
        print(x)
        qp.getPrimalSolution(x)
        obj = float(qp.getObjVal())

        print(x)
        print(obj)

        dt_ms = (time.perf_counter() - t0) * 1000.0
        _dlog(
            f"ok dt={dt_ms:.3f}ms obj={obj:.6g} nWSR={int(nWSR[0])} "
            f"x0_5={x[:min(5,nV)].tolist()}"
        )

        return {"status": "ok", "x": x.tolist(), "obj": obj, "nWSR": int(nWSR[0])}
    except Exception as e:
        dt_ms = (time.perf_counter() - t0) * 1000.0
        try:
            cmd = req.get("cmd")
            Hs = np.asarray(req.get("H", [])).shape
            As = np.asarray(req.get("A", [])).shape
            gs = np.asarray(req.get("g", [])).shape
            _dlog(f"err dt={dt_ms:.3f}ms cmd={cmd} H.shape={Hs} A.shape={As} g.shape={gs} exc={e}")
        except Exception:
            _dlog(f"err dt={dt_ms:.3f}ms exc={e}")
        return {"status": "err", "message": str(e)}


def main() -> int:
    if zmq is None:
        _err("pyzmq not installed; install 'pyzmq' to use qpoases-zmq server")
        return 2

    try:
        from qpoases import PySQProblem as SQProblem
        from qpoases import PyOptions as Options
        from qpoases import PyPrintLevel as PrintLevel
    except Exception as e:
        _err(f"failed to import qpoases: {e}")
        return 2

    options = Options()
    try:
        options.setToMPC()
    except Exception:
        pass
    try:
        options.printLevel = PrintLevel.PL_MEDIUM
    except Exception:
        pass

    qp_state = {
        "qp": None,
        "QProblem": SQProblem,
        "options": options,
    }

    # 仅保留 ZMQ 模式：默认绑定 tcp://127.0.0.1:5555
    addr = os.environ.get("QPOASES_IPC_ADDR", "tcp://127.0.0.1:5555")
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(addr)
    # sys.stderr.write(f"[qpoases-zmq] REP bound at {addr}\n")
    # sys.stderr.flush()

    while True:
        try:
            msg = sock.recv_string()
        except Exception as e:
            sys.stderr.write(f"[qpoases-zmq] recv error: {e}\n")
            sys.stderr.flush()
            break

        try:
            req = json.loads(msg)
        except Exception as e:
            resp = {"status": "err", "message": f"bad json: {e}"}
            sock.send_string(json.dumps(resp))
            continue

        resp = _handle_request(req, qp_state=qp_state)
        try:
            sock.send_string(json.dumps(resp))
        except Exception as e:
            sys.stderr.write(f"[qpoases-zmq] send error: {e}\n")
            sys.stderr.flush()
            break

        if req.get("cmd") == "close":
            break

    try:
        sock.close(0)
    except Exception:
        pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
