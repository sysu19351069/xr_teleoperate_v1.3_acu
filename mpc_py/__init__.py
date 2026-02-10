"""Python 版本的 MPC(QP) 迁移实现。

目前包含：
- lifted.py: 构造 Theta/Psi/Theta_plus、theta/omega/psi、Gq_dot 等常量矩阵
- mpc2qp.py: 构造 LAR/LSR 两阶段 QP（H,g,G,lbA,ubA）

后续会补充：
- OSQP 求解器封装
- Pinocchio 模型/Jacobian/IK 对接
- qpoases ipc server 模块入口（可选）
"""
