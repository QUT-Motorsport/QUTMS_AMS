#include "SoCEKF.h"

void SoC_reset()
{
	AMS_NVMem_t mem = {
			.SoC = 1.0f,
			.imp1 = 69,
			.imp2 = 420
	};
	flash_push(mem);
}

float SoC_pull()
{
	AMS_NVMem_t mem;
	flash_pull(&mem);
	return mem.SoC;
}

void SoC_push(float SoC)
{
	AMS_NVMem_t mem;
	flash_pull(&mem);
	mem.SoC = SoC;

	flash_push(mem);
}
