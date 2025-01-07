#pragma once 

// Ethercat Master---- 
#include "ecrt.h" 

// Vendor ID & Product Code 
#define Neuromeka 0x089A 
#define Neuromeka_NRMK_Drive 0x30000000 

extern ec_pdo_entry_info_t Neuromeka_NRMK_Drive_pdo_entries[]; 

