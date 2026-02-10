#include <iostream>
#include <vector>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <dlfcn.h>
#include <fstream>
#include <iomanip>

typedef int (*HRIF_Connect_t)(unsigned int, const char*, unsigned short);

typedef int (*HRIF_DisConnect_t)(unsigned int);

//extern int libshared_get_value();

int main(int argc, char *argv[])
{
    void* handle = dlopen("/usr/local/lib/libHR_Pro.so", RTLD_LAZY);
	if (handle == NULL) {
		printf("dlopen failed: %s\n", dlerror());
		return 1;
	}
	
	HRIF_Connect_t HRIF_Connect = (int(*)(unsigned int, const char*, short unsigned int))dlsym(handle, "_Z12HRIF_ConnectjPKct");
	
	//HRIF_DisConnect_t HRIF_DisConnect = (HRIF_DisConnect_t)dlsym(handle, "HRIF_DisConnect");
	
	if (HRIF_Connect == NULL) {
		printf("plugin_function not found\n");
		return 1;
	}
	
	//printf("main: value = %d\n", libshared_get_value());

    return 0;
}
