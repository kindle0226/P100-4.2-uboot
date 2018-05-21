/*  GIMP header image file format (RGB): C:\work\jorjin-c1-plan\character_img\9.h  */
/*  Call this macro repeatedly.  After each use, the pixel data can be extracted  */

#define HEADER_PIXEL(data,pixel) {\
pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4)); \
pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2)); \
pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33))); \
data += 4; \
}
static char *character_9_image_data =
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!7&B8H*S<O,CXU>(1U>(1KKKJD9W-1E*\"!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!D9W-[?HI```_```_"
	"```_```_```_```_```_```_O,CX1E*\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!*C9GR=8%```_```_```_```_```_```_```_```_```_```_```_^04TD9W-"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!*C9GX>X=```_```_```_```_```_```_```_"
	"```_```_```_```_```_```_```_D9W-!!!!!!!!!!!!!!!!!!!!!!!!KKKJ```_"
	"```_```_```_R=8%;WNL!!!!!!!!!!!!*C9GH*S<^04T```_```_```_```_;WNL"
	"!!!!!!!!!!!!!!!!;WNL```_```_```_```_H*S<!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!1E*\"X>X=```_```_```_^04T*C9G!!!!!!!!!!!!R=8%```_```_```_U>(1"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1E*\"^04T```_```_```_H*S<!!!!"
	"!!!!!!!!```_```_```_```_;WNL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!D9W-```_```_```_^04T!!!!!!!!;WNL```_```_```_^04T!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![?HI```_```_```_@8V]!!!!H*S<"
	"```_```_```_U>(1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"KKKJ```_```_```_KKKJ!!!!H*S<```_```_```_KKKJ!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!;WNL```_```_```_U>(1!!!!H*S<```_```_"
	"```_H*S<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!;WNL```_"
	"```_```_```_!!!!H*S<```_```_```_H*S<!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!;WNL```_```_```_```_!!!!H*S<```_```_```_U>(1"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!;WNL```_```_```_"
	"```_7&B8@8V]```_```_```_[?HI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!H*S<```_```_```_```_;WNL1E*\"```_```_```_```_7&B8!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*C9G[?HI```_```_```_```_;WNL"
	"!!!!U>(1```_```_```_R=8%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"O,CX```_```_```_```_```_;WNL!!!!;WNL```_```_```_```_O,CX!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!*C9GU>(1```_```_```_```_```_```_7&B8!!!!!!!!"
	"R=8%```_```_```_```_[?HIH*S<;WNL;WNL;WNL@8V]O,CX^04T```_```_KKKJ"
	"```_```_```_```_!!!!!!!!!!!!*C9GX>X=```_```_```_```_```_```_```_"
	"```_```_```_```_```_KKKJD9W-```_```_```_```_!!!!!!!!!!!!!!!!*C9G"
	"R=8%```_```_```_```_```_```_```_```_```_```_KKKJ!!!!H*S<```_```_"
	"```_^04T!!!!!!!!!!!!!!!!!!!!!!!!;WNLR=8%^04T```_```_```_```_```_"
	"R=8%;WNL!!!!!!!!KKKJ```_```_```_U>(1!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!7&B8;WNL;WNL7&B8!!!!!!!!!!!!!!!!!!!!U>(1```_```_```_O,CX"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!^04T```_```_```_D9W-!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!7&B8```_```_```_```_7&B8!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!KKKJ"
	"```_```_```_[?HI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!*C9G^04T```_```_```_KKKJ!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!H*S<```_```_```_"
	"```_1E*\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!7&B8```_```_```_```_O,CX!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!7&B8^04T```_```_```_^04T1E*\"!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!D9W-^04T"
	"```_```_```_```_D9W-!!!!!!!!!!!!!!!!!!!!!!!!!!!!H*S<;WNL!!!!!!!!"
	"!!!!*C9G;WNLH*S<[?HI```_```_```_```_```_KKKJ!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!```_```_```_```_```_```_```_```_```_```_```_```_```_"
	"D9W-!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!```_```_```_```_```_```_"
	"```_```_```_```_```_U>(17&B8!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!```_```_```_```_```_```_```_```_[?HIO,CX;WNL!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*C9G;WNL;WNL;WNL;WNL;WNL;WNL*C9G"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
