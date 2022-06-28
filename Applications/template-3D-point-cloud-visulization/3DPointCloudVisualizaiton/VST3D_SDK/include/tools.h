#pragma once

typedef unsigned char BYTE;

BYTE* Read8BitBmpFile2Img(const char * filename, int *width, int *height);
bool  Write8BitImg2BmpFile(BYTE *pImg, int width, int height, const char * filename);

BYTE* Read24BitBmpFile2Img(const char * filename, int *width, int *height);
bool  Write24BitImg2BmpFile(BYTE *pImg, int width, int height, const char * filename);
BOOL CtrlHandler(DWORD fdwCtrlType);
