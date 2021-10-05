#ifndef BACKPROCESS_EXPORT_H
#define BACKPROCESS_EXPORT_H

#define BACKPROCESS_LIBRARY

#ifdef _WIN64
#ifdef BACKPROCESS_LIBRARY
#define BACKPROCESSCOM_EXPORT __declspec(dllexport)
#else
#define BACKPROCESS_EXPORT __declspec(dllimport)
#endif
#else
#define BACKPROCESS_EXPORT
#endif


/* Define NULL pointer value */

#ifndef NULL
#ifdef  __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif

class BackProcessCom;

// 导出类
extern "C" BACKPROCESSCOM_EXPORT BackProcessCom *getBackProcessCom();
// 导出方法
extern "C" BACKPROCESSCOM_EXPORT const char *getVersion();

#endif //BACKPROCESS_EXPORT_H