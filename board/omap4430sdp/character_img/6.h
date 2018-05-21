/*  GIMP header image file format (RGB): C:\work\jorjin-c1-plan\character_img\6.h  */
/*  Call this macro repeatedly.  After each use, the pixel data can be extracted  */

#define HEADER_PIXEL(data,pixel) {\
pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4)); \
pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2)); \
pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33))); \
data += 4; \
}
static char *character_6_image_data =
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!;WNLH*S<H*S<U>(1U>(1"
	"U>(1O,CXH*S<@8V]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1E*\""
	"O,CX^04T```_```_```_```_```_```_```_```_U>(1!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!*C9GKKKJ```_```_```_```_```_```_```_```_```_```_"
	"```_U>(1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1E*\"X>X=```_```_```_```_"
	"```_```_[?HIU>(1U>(1U>(1U>(1^04TU>(1!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"*C9G[?HI```_```_```_```_[?HIH*S<1E*\"!!!!!!!!!!!!!!!!!!!!!!!!1E*\""
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!X>X=```_```_```_```_KKKJ!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!H*S<```_```_"
	"```_```_D9W-!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!*C9G^04T```_```_```_KKKJ!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!H*S<```_```_```_^04T*C9G"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!X>X=```_```_```_H*S<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!1E*\"```_```_```_```_*C9G!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!H*S<```_"
	"```_```_U>(1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!R=8%```_```_```_H*S<!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!X>X=```_```_```_"
	"@8V]!!!!!!!!!!!!!!!!7&B8;WNL;WNL;WNL;WNL!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!```_```_```_```_7&B8!!!!*C9GH*S<^04T```_```_```_```_"
	"```_```_R=8%;WNL!!!!!!!!!!!!!!!!!!!!1E*\"```_```_```_```_!!!!7&B8"
	"[?HI```_```_```_```_```_```_```_```_```_```_KKKJ!!!!!!!!!!!!!!!!"
	";WNL```_```_```_```_*C9G^04T```_```_```_^04TU>(1U>(1```_```_```_"
	"```_```_```_O,CX!!!!!!!!!!!!;WNL```_```_```_```_U>(1```_```_O,CX"
	"7&B8!!!!!!!!!!!!!!!!7&B8R=8%```_```_```_```_H*S<!!!!!!!!;WNL```_"
	"```_```_```_```_^04T;WNL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!O,CX```_"
	"```_```_```_1E*\"!!!!;WNL```_```_```_```_```_;WNL!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!![?HI```_```_```_H*S<!!!!@8V]```_```_```_"
	"```_O,CX!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!D9W-```_```_"
	"```_X>X=!!!!;WNL```_```_```_```_7&B8!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!7&B8```_```_```_```_!!!!;WNL```_```_```_```_!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!```_```_```_```_"
	"1E*\";WNL```_```_```_```_!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!```_```_```_```_;WNL*C9G```_```_```_```_!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!```_```_```_```_;WNL!!!!"
	"^04T```_```_```_1E*\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!```_```_```_```_1E*\"!!!!R=8%```_```_```_D9W-!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!1E*\"```_```_```_```_!!!!!!!!D9W-```_"
	"```_```_U>(1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!D9W-```_"
	"```_```_X>X=!!!!!!!!*C9G```_```_```_```_;WNL!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!U>(1```_```_```_KKKJ!!!!!!!!!!!!O,CX```_```_"
	"```_^04T*C9G!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!;WNL```_```_```_```_"
	"7&B8!!!!!!!!!!!!*C9G^04T```_```_```_[?HI7&B8!!!!!!!!!!!!!!!!!!!!"
	"!!!!;WNL^04T```_```_```_U>(1!!!!!!!!!!!!!!!!!!!!@8V]```_```_```_"
	"```_```_O,CX@8V];WNL;WNLD9W-O,CX```_```_```_```_^04T1E*\"!!!!!!!!"
	"!!!!!!!!!!!!!!!!D9W-```_```_```_```_```_```_```_```_```_```_```_"
	"```_```_^04T@8V]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!@8V]^04T```_```_"
	"```_```_```_```_```_```_```_```_[?HI7&B8!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!*C9GH*S<[?HI```_```_```_```_```_```_[?HIH*S<*C9G"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1E*\";WNL"
	"@8V];WNL;WNL1E*\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";