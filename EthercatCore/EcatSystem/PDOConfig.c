#include "PDOConfig.h" 
ec_pdo_entry_info_t Neuromeka_NRMK_Drive_pdo_entries[] = 
{ 
	{0x6040,	0,	16},	/* Controlword */
	{0x607a,	0,	32},	/* Targetposition */
	{0x60ff,	0,	32},	/* Targetvelocity */
	{0x6071,	0,	16},	/* Targettorque */
	{0x6060,	0,	8},	/* Modesofoperation */
	{0x6041,	0,	16},	/* Statusword */
	{0x6064,	0,	32},	/* Positionactualvalue */
	{0x606c,	0,	32},	/* Velocityactualvalue */
	{0x6077,	0,	16},	/* Torqueactualvalue */
	{0x6061,	0,	8},	/* Modesofoperationdisplay */
};

