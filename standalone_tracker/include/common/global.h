#ifndef _GLOBAL_H_
#define _GLOBAL_H_

namespace people {
	typedef enum {
		ObjNone = -1,
		ObjPerson,
		ObjCar,
		ObjTypeNum,
	}ObjectType;
	
	extern ObjectType g_objtype;
};

#endif
