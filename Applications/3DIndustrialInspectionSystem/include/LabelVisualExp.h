#ifndef LABELVISUAL_EXPORT_H
#define LABELVISUAL_EXPORT_H

#define LABELVISUAL_LIBRARY

#ifdef _WIN64
#ifdef LABELVISUAL_LIBRARY
#define LABELVISUALCOM_EXPORT __declspec(dllexport)
#else
#define LABELVISUALCOM_EXPORT __declspec(dllimport)
#endif
#else
#define LABELVISUALCOM_EXPORT
#endif


/* Define NULL pointer value */

#ifndef NULL
#ifdef  __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif

class LabelVisualCom;

// 导出类
extern "C" LABELVISUALCOM_EXPORT LabelVisualCom *getLabelVisualCom();
// 导出方法
extern "C" LABELVISUALCOM_EXPORT const char *getVersion();

#endif //LABELVISUAL_EXPORT_H