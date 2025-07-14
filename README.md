A VEHICL with a CAMERA


Purpose: Collect and analyze camera information

chipset: nxp  IMX8mp

version:
          kernel 5.10.72.2.2.0

          u-boot 2021.04

 u-boot

1) Unlock the compressed file(u-boot-imx_20240913.tar.gz)

  $mkdir u-boot-imx
  
  $cd   cd u-boot-imx
  
  $tar cvfz u-boot-imx_20240913.tar.gz

2) u-boot build
 
  $ ./uboot_make.sh imx8mp_mv_defconfig

  $ ./uboot_make.sh 

======================================================================

Kernel

1) Unlock the compressed file(linux-imx_20240913.tar.gz)
  
  $mkdir linux-imx 

  $tar cvfz linux-imx_20240913.tar.gz
  
2) kernel build

 $ ./kernel_make.sh mv_v8_defconfig
 
 $ ./kernel_make.sh

======================================================================

eosys_drv_m.c  ==> infrared camera driver source code

mdin_drv_m.c   ==> cmos camera driver source code

tp2860_drv_m.c ==> auxiliary camera driver source code






