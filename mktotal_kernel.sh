#!/bin/bash
root_path=$(pwd)

if [ "$1" == "VKI" ]
then
	echo "Use $1"
	configed=1
elif [ "$1" == "RRI_WIFI" ]
then
	echo "Use $1"
	configed=1
elif [ "$1" == "RRI_WIFI_BT" ]
then
	echo "Use $1"
	configed=1
#+++wwj begin 20150313@add HIO compile
elif [ "$1" == "HIO" ]
then
	echo "Use $1"
	configed=1
#++++wwj end 20150313@add HIO compile
#+++wwj begin 20150316@add THIN_BOX compile
elif [ "$1" == "THIN_BOX" ]
then
	echo "Use $1"
	configed=1
#++++wwj end 20150316@add THIN_BOX compile
#+++wwj begin 20150422@add POE compile
elif [ "$1" == "POE" ]
then
	echo "Use $1"
	configed=1
#++++wwj end 20150422@add POE compile

else
	echo "----------------------------------------------------------------"
	echo "Please choose below type:"
    	echo "VKI"
	echo "RRI_WIFI"
	echo "RRI_WIFI_BT"
#+++wwj 20150313@add HIO compile
        echo "HIO"
#+++wwj 20150316@add THIN_BOX compile
        echo "THIN_BOX"
#+++wwj 20150433@add POE compile
        echo "POE"
#+++wwj 20150422@add POE compile

	echo "Example:"
	echo "        ./mktotal.sh VKI"
    configed=0
fi

TYPE_FILE=./include/linux/SHEN_TYPE.h
if [ "$configed" == "1" ] 
then
	echo "----------------------------------------------------------------"
	perl -pi -e 's/\#define CUR_SHEN_TYPE .*/\#define CUR_SHEN_TYPE TYPE_'"$1"'/g' $TYPE_FILE
	cat $TYPE_FILE | grep "CUR_SHEN_TYPE"
	echo "----------------------------------------------------------------"
	make ARCH=arm CROSS_COMPILE=arm-eabi- distclean
	
	if [ "$1" == "VKI" ]
	then
		make ARCH=arm CROSS_COMPILE=arm-eabi- imx_v7_vki_defconfig
	elif [ "$1" == "RRI_WIFI" ]
	then
		make ARCH=arm CROSS_COMPILE=arm-eabi- imx_v7_defconfig
	elif [ "$1" == "RRI_WIFI_BT" ]
	then
		make ARCH=arm CROSS_COMPILE=arm-eabi- imx_v7_bt_defconfig
#++++wwj begin 20150313@add HIO compile
	elif [ "$1" == "HIO" ]
	then
		make ARCH=arm CROSS_COMPILE=arm-eabi- imx_v7_hio_defconfig
#++++wwj end 20150313@add HIO compile
#++++wwj begin 20150316@add THIN_BOX compile
	elif [ "$1" == "THIN_BOX" ]
	then
		make ARCH=arm CROSS_COMPILE=arm-eabi- imx_v7_hio_defconfig
#++++wwj end 20150316@add THIN_BOX compile
#++++wwj begin 20150422@add POE compile
	elif [ "$1" == "POE" ]
	then
		make ARCH=arm CROSS_COMPILE=arm-eabi- imx_v7_vki_defconfig
#++++wwj end 20150422@add POE compile

	fi
	echo "Start compile $1 ......"
	if [ -f arch/arm/boot/uImage ]
	then
		echo "Remove old uImage"
		rm -rf arch/arm/boot/uImage
		echo [Done]
		echo "----------------------------------------------------------------"
	fi
	
	if [ -f dts/imx6dl-sabresd.dtb ]
	then
		echo "Remove old uImage"
		rm -rf dts/imx6dl-sabresd.dtb
		echo [Done]
		echo "----------------------------------------------------------------"
	fi

	#make ARCH=arm CROSS_COMPILE=arm-eabi- uImage -j16
	make uImage -j36 LOADADDR=0x10008000
	make dtbs
	echo "----------------------------------------------------------------"

	echo "Create new uImage_$1"
	cp arch/arm/boot/uImage   arch/arm/boot/uImage_$1_$(date +"%Y%m%d%H")
	cp dts/imx6dl-sabresd.dtb dts/imx6dl-sabresd_$1_$(date +"%Y%m%d%H").dtb
	echo [Done]
	echo "----------------------------------------------------------------"
	echo "New image files md5sum:"
	mv arch/arm/boot/uImage_$1_$(date +"%Y%m%d%H") ./
	mv dts/imx6dl-sabresd_$1_$(date +"%Y%m%d%H").dtb ./
	md5sum  ./uImage_$1_$(date +"%Y%m%d%H") > ./uImage_$1_$(date +"%Y%m%d%H")_md5
	md5sum  ./imx6dl-sabresd_$1_$(date +"%Y%m%d%H").dtb > ./imx6dl-sabresd_$1_$(date +"%Y%m%d%H").dtb_md5
	echo [Done]
fi
