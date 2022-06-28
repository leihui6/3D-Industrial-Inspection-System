#include "stdafx.h"
#include "tools.h"


BYTE *Read8BitBmpFile2Img(const char * filename, int *width, int *height)
{
	FILE *BinFile;
	BITMAPFILEHEADER FileHeader;
	BITMAPINFOHEADER BmpHeader;
	BYTE *pImg;
	unsigned int size;
	int Suc = 1, w, h;


	// Open File
	*width = *height = 0;
	if ((BinFile = fopen(filename, "rb")) == NULL) return NULL;
	// Read Struct Info
	if (fread((void *)&FileHeader, 1, sizeof(FileHeader), BinFile) != sizeof(FileHeader)) Suc = -1;
	if (fread((void *)&BmpHeader, 1, sizeof(BmpHeader), BinFile) != sizeof(BmpHeader)) Suc = -1;
	if ((Suc == -1) ||
		(FileHeader.bfOffBits < sizeof(FileHeader) + sizeof(BmpHeader))
		)
	{
		fclose(BinFile);
		return NULL;
	}

	// Read Image Data
	*width = w = (BmpHeader.biWidth + 3) / 4 * 4;
	*height = h = BmpHeader.biHeight;
	size = (BmpHeader.biWidth + 3) / 4 * 4 * BmpHeader.biHeight;
	fseek(BinFile, FileHeader.bfOffBits, SEEK_SET);
	if ((pImg = new BYTE[size]) != NULL)
	{
		for (int i = 0; i < h; i++)  // 0,1,2,3,4(5): 400-499
		{
			if (fread(pImg + (h - 1 - i)*w, sizeof(BYTE), w, BinFile) != w)
			{
				fclose(BinFile);
				delete pImg;
				pImg = NULL;
				return NULL;
			}
		}
	}
	fclose(BinFile);
	return pImg;
}

bool Write8BitImg2BmpFile(BYTE *pImg, int width, int height, const char * filename)
// 当宽度不是4的倍数时自动添加成4的倍数
{
	FILE * BinFile;
	BITMAPFILEHEADER FileHeader;
	BITMAPINFOHEADER BmpHeader;
	int i, extend;
	bool Suc = true;
	BYTE p[4], *pCur;

	// Open File
	if ((BinFile = fopen(filename, "w+b")) == NULL) { return false; }
	// Fill the FileHeader
	FileHeader.bfType = ((WORD)('M' << 8) | 'B');
	FileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + 256 * 4L;
	FileHeader.bfSize = FileHeader.bfOffBits + width*height;
	FileHeader.bfReserved1 = 0;
	FileHeader.bfReserved2 = 0;
	if (fwrite((void *)&FileHeader, 1, sizeof(FileHeader), BinFile) != sizeof(FileHeader)) Suc = false;
	// Fill the ImgHeader
	BmpHeader.biSize = 40;
	BmpHeader.biWidth = width;
	BmpHeader.biHeight = height;
	BmpHeader.biPlanes = 1;
	BmpHeader.biBitCount = 8;
	BmpHeader.biCompression = 0;
	BmpHeader.biSizeImage = 0;
	BmpHeader.biXPelsPerMeter = 0;
	BmpHeader.biYPelsPerMeter = 0;
	BmpHeader.biClrUsed = 0;
	BmpHeader.biClrImportant = 0;
	if (fwrite((void *)&BmpHeader, 1, sizeof(BmpHeader), BinFile) != sizeof(BmpHeader)) Suc = false;
	// write Pallete
	for (i = 0, p[3] = 0; i < 256; i++)
	{
		p[3] = 0;
		p[0] = p[1] = p[2] = i; // blue,green,red;
		if (fwrite((void *)p, 1, 4, BinFile) != 4) { Suc = false; break; }
	}
	// write image data
	extend = (width + 3) / 4 * 4 - width;
	if (extend == 0)
	{
		for (pCur = pImg + (height - 1)*width; pCur >= pImg; pCur -= width)
		{
			if (fwrite((void *)pCur, 1, width, BinFile) != (unsigned int)width) Suc = false; // 真实的数据
		}
	}
	else
	{
		for (pCur = pImg + (height - 1)*width; pCur >= pImg; pCur -= width)
		{
			if (fwrite((void *)pCur, 1, width, BinFile) != (unsigned int)width) Suc = false; // 真实的数据
			for (i = 0; i < extend; i++) // 扩充的数据
				if (fwrite((void *)(pCur + width - 1), 1, 1, BinFile) != 1) Suc = false;
		}
	}
	// return;
	fclose(BinFile);
	return Suc;
}

BYTE *Read24BitBmpFile2Img(const char * filename, int *width, int *height)
{
	FILE * BinFile;
	BITMAPFILEHEADER FileHeader;
	BITMAPINFOHEADER BmpHeader;
	BYTE *img;
	unsigned int size;
	int Suc = 1, w, h;

	// Open File
	*width = *height = 0;
	if ((BinFile = fopen(filename, "rb")) == NULL) return NULL;
	// Read Struct Info
	if (fread((void *)&FileHeader, 1, sizeof(FileHeader), BinFile) != sizeof(FileHeader)) Suc = -1;
	if (fread((void *)&BmpHeader, 1, sizeof(BmpHeader), BinFile) != sizeof(BmpHeader)) Suc = -1;
	if ((Suc == -1) ||
		(FileHeader.bfOffBits < sizeof(FileHeader) + sizeof(BmpHeader))
		)
	{
		fclose(BinFile);
		return NULL;
	}
	// Read Image Data
	*width = w = (BmpHeader.biWidth + 3) / 4 * 4;
	*height = h = BmpHeader.biHeight;
	size = (*width)*(*height) * 3;
	fseek(BinFile, FileHeader.bfOffBits, SEEK_SET);
	if ((img = new BYTE[size]) != NULL)
	{
		for (int i = 0; i < h; i++)
		{
			if (fread(img + (h - 1 - i)*w * 3, sizeof(BYTE), w * 3, BinFile) != w * 3)
			{
				fclose(BinFile);
				delete img;
				img = NULL;
				return NULL;
			}
		}
	}
	fclose(BinFile);
	return img;
}
bool Write24BitImg2BmpFile(BYTE *pImg, int width, int height, const char * filename)
// 当宽度不是4的倍数时自动添加成4的倍数
{
	FILE *BinFile;
	BITMAPFILEHEADER FileHeader;
	BITMAPINFOHEADER BmpHeader;
	bool Suc = true;
	int i, extend;
	BYTE *pCur;

	// Open File
	if ((BinFile = fopen(filename, "w+b")) == NULL) { return false; }
	// Fill the FileHeader
	FileHeader.bfType = ((WORD)('M' << 8) | 'B');
	FileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	FileHeader.bfSize = FileHeader.bfOffBits + width*height * 3L;
	FileHeader.bfReserved1 = 0;
	FileHeader.bfReserved2 = 0;
	if (fwrite((void *)&FileHeader, 1, sizeof(BITMAPFILEHEADER), BinFile) != sizeof(BITMAPFILEHEADER)) Suc = false;
	// Fill the ImgHeader
	BmpHeader.biSize = 40;
	BmpHeader.biWidth = width;
	BmpHeader.biHeight = height;
	BmpHeader.biPlanes = 1;
	BmpHeader.biBitCount = 24;
	BmpHeader.biCompression = 0;
	BmpHeader.biSizeImage = 0;
	BmpHeader.biXPelsPerMeter = 0;
	BmpHeader.biYPelsPerMeter = 0;
	BmpHeader.biClrUsed = 0;
	BmpHeader.biClrImportant = 0;
	if (fwrite((void *)&BmpHeader, 1, sizeof(BITMAPINFOHEADER), BinFile) != sizeof(BITMAPINFOHEADER)) Suc = false;
	// write image data
	extend = (width + 3) / 4 * 4 - width;
	if (extend == 0)
	{
		for (pCur = pImg + (height - 1) * 3 * width; pCur >= pImg; pCur -= 3 * width)
		{
			if (fwrite((void *)pCur, 1, width * 3, BinFile) != (unsigned int)(3 * width)) Suc = false; // 真实的数据
		}
	}
	else
	{
		for (pCur = pImg + (height - 1) * 3 * width; pCur >= pImg; pCur -= 3 * width)
		{
			if (fwrite((void *)pCur, 1, width * 3, BinFile) != (unsigned int)(3 * width)) Suc = false; // 真实的数据
			for (i = 0; i < extend; i++) // 扩充的数据
			{
				if (fwrite((void *)(pCur + 3 * (width - 1) + 0), 1, 1, BinFile) != 1) Suc = false;
				if (fwrite((void *)(pCur + 3 * (width - 1) + 1), 1, 1, BinFile) != 1) Suc = false;
				if (fwrite((void *)(pCur + 3 * (width - 1) + 2), 1, 1, BinFile) != 1) Suc = false;
			}
		}
	}
	// return;
	fclose(BinFile);
	return Suc;
}
BOOL CtrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
		// Handle the CTRL+C signal. 

	case CTRL_C_EVENT:

		Beep(1000, 1000);
		return TRUE;

		// CTRL+CLOSE: confirm that the user wants to exit. 

	case CTRL_CLOSE_EVENT:

		return TRUE;

		// Pass other signals to the next handler. 

	case CTRL_BREAK_EVENT:

	case CTRL_LOGOFF_EVENT:

	case CTRL_SHUTDOWN_EVENT:

	default:

		return FALSE;
	}
}


