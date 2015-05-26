#ifndef __SHENONMXC_H

#define __SHENONMXC_H

#include "SHEN_TYPE.h"


#if ((CUR_SHEN_TYPE) == (TYPE_VKI)) 


#elif ((CUR_SHEN_TYPE) == (TYPE_RRI_WIFI)) || ((CUR_SHEN_TYPE) == (TYPE_RRI_WIFI_BT))
#define CONFIG_RII_USBHUB_STAT_PWR
#define CONFIG_RII
#define CONFIG_PWR_CD
#define CONFIG_RII_DDR_CLL
#define CONFIG_WIFI_IC

//+++wwj
//#define CONFIG_WIFI_BT

//+++wwj begin 20150313@add HIO compile
#elif ((CUR_SHEN_TYPE) == (TYPE_HIO)) 

//+++wwj end 20150313
//+++wwj begin 20150316@add THIN_BOX compile
#elif ((CUR_SHEN_TYPE) == (TYPE_THIN_BOX)) 

//+++wwj end 20150316
#define CONFIG_NO_FLEXCAN1
#define CONFIG_THIN_BOX

//+++wwj 20150317@add the mic detect
#define CONFIG_MIC_DEC

//+++wwj begin 20150422@add POE compile
#elif ((CUR_SHEN_TYPE) == (TYPE_POE)) 
#define CONFIG_POE_UART3
#define CONFIG_POE_UART4
#define CONFIG_POE_UART5

#endif
#endif
