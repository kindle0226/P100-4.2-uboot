/*  GIMP header image file format (RGB): /home/karl/shm/LOGO/C1_Fastboot-2.h

static unsigned int width = 188;
static unsigned int height = 39;
  */
/*  Call this macro repeatedly.  After each use, the pixel data can be extracted  */

#define HEADER_PIXEL(data,pixel) {\
pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4)); \
pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2)); \
pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33))); \
data += 4; \
}
static char *fastboot_mid_image_data =
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"3$UZH'.)B'\"0?FV/?FV/?FZ/?FZ/?FZ/?FZ/?FZ0?FZ0?FZ0?FZ0?FZ0?FZ0?F^0"
	"?F^0?F^0?F^0H7B+=&J.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!@'B6N9\"-VHUNX8I?X8M?X8M?X8QAU))XKX^3"
	"9V>-!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!96\")UD1>UD1>UD9>"
	"UT=>UTE>UTI>UTI>UTM>UTQ>UTY>UT]>UT]>UU%>UU%>V%)?V%-?V%1?V%5?V%5?"
	"H7F,!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!6%J$RX]^X(A@X(E@X(I@X8I?X8M?X8M?X8QAX8UAX8YAX8]ANI*.)S%B!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!96\")UD1>UD1>UD9>UT=>UTE>UTI>UTI>"
	"UTM>UTQ>UTY>UT]>UT]>UU%>UU%>V%)?V%-?V%1?V%5?V%5?H7F,!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$UYVHMMX(=?X(A@"
	"X(E@X(I@X8I?X8M?X8M?X8QAX8UAX8YAX8]AX8]AS):!)S%B!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!9VV1:&V1:&V1:&V12$]["
	"!!!!!!!!!!!!!!!!96\")UD1>UD1>UD9>UT=>UTE>UTI>UTI>UTM>UTQ>UTY>UT]>"
	"UU!>UU%>UU%>V%)?V%-?V%1?V%5?V%5?H7F,!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PH^&X(9@X(=?X(A@X(E@X(I@VXUMPY*'"
	"RY)_X8QAX8UAX8YAX8]AX8]AXI!AI8V8!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!![L1O[\\5O[\\9O[\\=OD8^A!!!!!!!!!!!!!!!!"
	"6%:\"J7*%E7..E7..E7..E7..E72.E72.NVQYUTQ>UTY>UT]>RV-LEG6/EG6/EG:/"
	"EG:/EG:/EG>0H7B,B'21!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!9F:,X(9@X(9@X(=?X(A@VHQN='&3!!!!!!!!!!!!)S%BF8>9X8YA"
	"X8]AX8]AXI!AXI!B2$UY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!![L1O[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F867"
	"X(9@X(9@X(=?X(A@6%J$!!!!!!!!!!!!!!!!!!!!!!!!C8&9X8]AX8]AXI!AXI!B"
	"C8*:!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27"
	"[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PH^&X(9@X(9@X(=?PI\"'"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!W))OX8]AXI!AXI!BPY6)!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>"
	"O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!9F.*JWN(S6YNV6!?V6%?VF%?SG%OK'V*9F.+!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!2$QYEWV3EWV3EWV39F2+!!!!)S%BC7J5OX&!W')?W7-?W71?P(2\""
	"F'^42$QY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!PH^&X(9@X(9@X(=?PI\"'!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!@7J7F8>99V>-9V>-!!!!!!!!!!!!!!!!!!!!!!!!2$UZFHJ:"
	"FHJ:FHJ:@7R8!!!!)S%B@7R8Q9N,UIY[WIYQQ9V,FHR;)S%B!!!!!!!!!!!!2$YZ"
	"II2;Q:\"-X*5SX*5SQ:\"-CX><)S%B!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!@H\":O:.7XJ]VZ*]JZ;!JZ;%JZ;%JT:V*LZ\"<2$YZ!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!G)2AEI\"A@H*<@H*<.$%O!!!!.$%OC8J?J9^AJ9^AJ9^AJ9^B"
	"7F.+!!!!!!!!65Z'G9>CG9>CG9BCWKZ([\\5O[\\9O[\\=OM:FAG9FCG9FDJ:*D@X6>"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)S%BO79]V5Y@V5Y@V6!?"
	"V6!?V6%?VF%?VF)?VF-?VF-?M7V%)S%B!!!!!!!!!!!!!!!!!!!!!!!!!!!!9F2+"
	"W&U?W&Y?W&Y?EWV36%F#SWMQW'%?W'%?W')?W7-?W71?W71?W71?UWQJ@'64!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!KHR1X(9@X(9@X(=?VHQN)S%B!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!4%1_XY9CXY9CXY=CQ)F+6%N%"
	"UIQZY)IDY)MDY)MDY)QDY)UDY9YEWZ!R2$UZ!!!!@7V9WZ)RYJ)EYJ-GYJ-FYJ-F"
	"YJ1FYJ5FV*5^6%R&!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)S%BQZ:1Z*UHZ*UH"
	"Z*]HZ*]JZ;!JZ;%JZ;%JZK)JZK)JZK1K@H&;!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"R:Z4Z[ELZ[IL[+IL9VR0@X*<[+QM[+UM[+UM[;YM[;]M[<!N@X.<!!!!!!!!9VR1"
	"[L-N[L-N[L1O[L1O[\\5O[\\9O[\\=O[\\=O[\\=O[\\AP[\\EPG9FD!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!2$MXTV1FV5U@V5Y@V5Y@V6!?V6!?V6%?VF%?VF)?"
	"VF-?VF-?VF1?QG=V)S%B!!!!!!!!!!!!!!!!!!!!!!!!9F2+W&U?W&Y?W&Y?MH&'"
	"UG5HW'!?W'%?W'%?W')?W7-?W71?W71?W71?W79@W79@C7R6!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!C7^8"
	"X(9@X(9@X(=?X(A@RY!_9F>-!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CS9N#W9IQXYECY)IDY)MDY)MD"
	"Y)QDY)UDY9YEY9YFNYN3=724Y:!FY:%FYJ)EYJ-GYJ-FYJ-FYJ1FYJ5FYJ9FV*5^"
	")S%B!!!!!!!!!!!!!!!!!!!!!!!!)S%BV:I`Z*UHZ*UHZ*UHZ*]HZ*]JZ;!JZ;%J"
	"Z;%JZK)JZK)JZK1KZK1K=7>6!!!!!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL"
	"J)VA[+QM[+QM[+UM[+UM[;YM[;]M[<!N65Z'!!!!!!!!9VR1[L-N[L-N[L1O[L1O"
	"[\\5O[\\9O[\\=O[\\=O[\\=O[\\AP[\\EPG9FD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!Q7%UV5Q?V5U@V5Y@V5Y@TVAFJWN)EGJ1JWR)U&IFVF-?VF-?VF1?VF5?"
	"MGZ%!!!!!!!!!!!!!!!!!!!!!!!!8%^'W&U?W&Y?W&Y?W&]>W&]>W'!?W'%?T'QQ"
	"OX*!P(.!W71?W71?W71?W79@W79@W7=@6%F$!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)S%BVHMMX(9@X(=?X(A@"
	"X(E@X(I@RY%_I8N6='*3)S%B!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!2$UZXY9CXY9CXY=CXYACXYECXYECY)IDQ9N,SIV$Y)QDY)UDY9YEY9YF"
	"Y9YFWZ%RY:!FY:%FYJ)EV*-^Q:\"-YJ-FYJ1FYJ5FYJ9FYJ9F@GZ:!!!!!!!!!!!!"
	"!!!!!!!!!!!!LIR;Z*QHZ*UHZ*UHZ*UHLIZ;=7:69VN/9VN/@H\";X[%VZK)JZK1K"
	"ZK1KT:^,!!!!!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL[+ML[+QM[+QM[+UM"
	"[+UM[;YM[;]M[<!N!!!!!!!!!!!!9VR1[L-N[L-N[L1O[L1O[\\5O[\\9O[\\=O[\\=O"
	"[\\=O[\\AP[\\EPG9FD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>"
	"O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!?W\"1V5M?V5Q?"
	"V5U@V5Y@JWN()S%B!!!!!!!!!!!!)S%BK'V*VF-?VF1?VF5?VF9?9F.+!!!!!!!!"
	"!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?W&]>W&]>UG9I@'23!!!!!!!!!!!!)S%BK86."
	"W71?W79@W79@W7=@MX>(!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!@'B6X(9@X(=?X(A@X(E@X(I@X8I?X8M?"
	"X8M?X8QARY-`I8R76%N%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$UZXY9C"
	"XY9CXY=CXYACXYECS9R#2$UZ!!!!!!!!6%N%WIYQY9YEY9YFY9YFY:!FY:!FSZ&%"
	"=724!!!!!!!!6%R&X*5SYJ5FYJ9FYJ9FO)^5!!!!!!!!!!!!!!!!!!!!)S%BZ*MH"
	"Z*QHZ*UHZ*UHCXF>!!!!!!!!!!!!!!!!!!!!9VN0ZK)JZK1KZK1KZK1K!!!!!!!!"
	"!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL[+ML[+QMW;B%D(V@9VR09VR1G9:BJ9^B"
	"!!!!!!!!!!!!)S%B9VR1)S%B!!!!RK27[\\5O[\\9O[\\=O:&V1!!!!2$][:&V12$]["
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!S&MMV5M?V5Q?V5U@M7J#!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!QG9VVF1?VF5?VF9?MGZ%!!!!!!!!!!!!!!!!!!!!2$QY"
	"W&U?W&Y?W&Y?W&]>W&]>6%F#!!!!!!!!!!!!!!!!!!!!!!!!MX6(W79@W79@W7=@"
	"W7A@'BE9!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!F867X(=?X(A@X(E@X(I@X8I?X8M?X8M?X8QAX8UAX8YA"
	"X8]APY2(6%N%!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CXYACW9IQ"
	")S%B!!!!!!!!!!!!!!!!II*:Y9YEY9YFY9YFY:!FUZ%])S%B!!!!!!!!!!!!!!!!"
	"II2;YJ5FYJ9FYJ9FQ:*/!!!!!!!!!!!!!!!!!!!!)S%BFY\">QZ:0V:M`V:Q`!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!X[%VZK1KZK1KZK1K)S%B!!!!!!!!!!!!!!!!!!!!"
	"R:Z4Z[ELZ[IL[+IL[+MLT[6.)S%B!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!)S%BV5I?V5M?V5Q?V5U@2$MX!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!<VN/VF1?VF5?VF9?VF=?!!!!!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?W&]>"
	"EWZ3!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$QYW79@W79@W7=@W7A@8V**!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!9F:,N8^-X(E@X(I@X8I?X8M?X8M?X8QAX8UAX8YAX8]AX8]AW)1O='*4"
	"!!!!!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CXYAC@7R8!!!!!!!!!!!!!!!!"
	"!!!!=724Y9YEY9YFY9YFY:!F@7V9!!!!!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9F"
	"Q:*/!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!R*J1ZK1KZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL"
	"[+ML2$YZ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27"
	"[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"4%%]V5I?V5M?V5Q?S6UN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!VF1?VF5?"
	"VF9?VF=?+SAH!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?W&]>2$QY!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!P(6#W79@W7=@W7A@@':4!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	")S%B='&3I8J6RY)_X8M?X8QAX8UAX8YAX8]AX8]AXI!AW)1P2$UY!!!!!!!!!!!!"
	"!!!!!!!!2$UZXY9CXY9CXY=CXYAC)S%B!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YF"
	"Y9YFY:!F)S%B!!!!!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9FQ:*/!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!)S%B9VN/CXF>G)*?IYF>R*F1R*J1X[%VZK1K"
	"ZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+ILM*2>!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>"
	"O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!9F**V5I?V5M?V5Q?"
	"M7F#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!OGI^VF5?VF9?VF=?)S%B!!!!"
	"!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?SWIQ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!F(\"5W79@W7=@W7A@@':4!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	")S%B@'F7N9*.X8YAX8]AX8]AXI!AXI!BKY&4!!!!!!!!!!!!!!!!!!!!2$UZXY9C"
	"XY9CXY=CS9N#!!!!!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YFY9YFSJ\"%!!!!!!!!"
	"!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9FQ:*/!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!9VJ/O:.7Z*UHZ*]HZ*]JZ;!JZ;%JZ;%JZK)JZK)JZK1KZK1KZK1K2$YZ!!!!"
	"!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+ILG)6A!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!9F**V5I?V5M?V5Q?EGF1!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!OGI^VF5?VF9?VF=?)S%B!!!!!!!!!!!!!!!!2$QY"
	"W&U?W&Y?W&Y?OX!`!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F(\"5W79@W7=@"
	"W7A@@':4!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!9V>-"
	"X8]AX8]AXI!AXI!BW)5P!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CQ)F+!!!!"
	"!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YFY9YFQ9^,!!!!!!!!!!!!!!!!!!!!!!!!"
	"9VF/YJ5FYJ9FYJ9FQ:*/!!!!!!!!!!!!!!!!!!!!!!!!!!!!LIV;Z*UHZ*UHZ*UH"
	"Z*]HZ*]JZ;!JZ;%JZ;%JZK)JZK)JZK1KZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!"
	"R:Z4Z[ELZ[IL[+IL9VR0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!9F**V5I?V5M?V5Q?H7J-!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!OGI^VF5?VF9?VF=?)S%B!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?R'YX"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!F(\"5W79@W7=@W7A@@':4!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!2$UY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!KY\"3X8]AXI!AXI!B"
	"XI%B!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CQ)F+!!!!!!!!!!!!!!!!!!!!"
	"!!!!9VF.Y9YEY9YFY9YFQ9^,!!!!!!!!!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9F"
	"Q:*/!!!!!!!!!!!!!!!!!!!!!!!!IY>=Z*QHZ*UHZ*UHZ*UHZ*]HT:N*LY^;G)*?"
	"CXJ?9VN0T:V*ZK1KZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL"
	"9VR0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27"
	"[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"7UV'V5I?V5M?V5Q?O79]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!SG-OVF5?"
	"VF9?VF=?)S%B!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?W&]>)S%B!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!MX6(W79@W7=@W7A@@':4!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!6%J$PH^&VHMMX(9@X(=?)S%B"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!='*3X8]AXI!AXI!BXI%B!!!!!!!!!!!!"
	"!!!!!!!!2$UZXY9CXY9CXY=CQ)F+!!!!!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YF"
	"Y9YFQ9^,!!!!!!!!!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9FQ:*/!!!!!!!!!!!!"
	"!!!!!!!!)S%BZ*MHZ*QHZ*UHZ*UHQZ:12$YZ!!!!!!!!!!!!!!!!!!!!T:V*ZK1K"
	"ZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL9VR0!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>"
	"O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$MXV5I?V5M?V5Q?"
	"V5U@)S%B!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$MXVF1?VF5?VF9?VF=?#1A)!!!!"
	"!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?W&]>@'23!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!UWQJW79@W7=@W7A@;6J.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!4%-_X(5@X(9@X(9@X(=?='&2!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!C8&9X8]AXI!AXI!BXI%B!!!!!!!!!!!!!!!!!!!!2$UZXY9C"
	"XY9CXY=CQ)F+!!!!!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YFY9YFQ9^,!!!!!!!!"
	"!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9FQ:*/!!!!!!!!!!!!!!!!!!!!8&.+Z*MH"
	"Z*QHZ*UHV:Q`!!!!!!!!!!!!!!!!!!!!!!!!!!!!ZK)JZK1KZK1KZK1K2$YZ!!!!"
	"!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL9VR0!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!V5I?V5M?V5Q?V5U@C':2!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!EWJ1VF1?VF5?VF9?QGEW!!!!!!!!!!!!!!!!!!!!2$QY"
	"W&U?W&Y?W&Y?W&]>SWMQ)S%B!!!!!!!!!!!!!!!!!!!!!!!!C7N5W79@W79@W7=@"
	"W7A@.#]N!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!)S%BX(5@X(9@X(9@X(=?PI\"'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"PY2(X8]AXI!AXI!BW)5P!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CQ)F+!!!!"
	"!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YFY9YFQ9^,!!!!!!!!!!!!!!!!!!!!!!!!"
	"9VF/YJ5FYJ9FYJ9FQ:*/!!!!!!!!!!!!!!!!!!!!9VJ/Z*MHZ*QHZ*UHFY\">!!!!"
	"!!!!!!!!!!!!!!!!!!!!9VN0ZK)JZK1KZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!"
	"R:Z4Z[ELZ[IL[+IL9VR0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!RK27[\\5O[\\9O[\\=O:&V1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!JWF(V5M?V5Q?V5U@TV9F6%B#!!!!!!!!!!!!!!!!!!!!"
	"<VN/VF-?VF1?VF5?VF9?C'B4!!!!!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?W&]>"
	"W&]>R']Y2$QY!!!!!!!!!!!!!!!!<VZ0W71?W79@W79@W7=@R81[!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RXY^"
	"X(9@X(9@X(=?X(A@N9\"-)S%B!!!!!!!!!!!!!!!!)S%BKX^3X8]AX8]AXI!AXI!B"
	"NI./!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CQ)F+!!!!!!!!!!!!!!!!!!!!"
	"!!!!9VF.Y9YEY9YFY9YFQ9^,!!!!!!!!!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9F"
	"Q:*/!!!!!!!!!!!!!!!!!!!!9VJ/Z*MHZ*QHZ*UHLIZ;!!!!!!!!!!!!!!!!!!!!"
	"6%V&X[%VZK)JZK1KZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL"
	"9VR0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27"
	"[\\5O[\\9O[\\=O=GF8!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"E72.UTQ>UTY>UT]>O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!2$MXV5M?V5Q?V5U@V5Y@TV9FEGF12$MX!!!!6%B#EWJ1VF-?VF-?VF1?VF5?"
	"U&YF)S%B!!!!!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?W&]>W&]>W'!?UG=IK(.-"
	"EWZ4F'^4P(2\"W71?W71?W79@W79@W7=@=&^1!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!='&2X(9@X(9@X(=?X(A@"
	"X(E@VHUNN9\"-F8:8F8:8KX^3VY%OX8YAX8]AX8]AXI!AXI!B9V>-!!!!!!!!!!!!"
	"!!!!!!!!2$UZXY9CXY9CXY=CQ)F+!!!!!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YF"
	"Y9YFQ9^,!!!!!!!!!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9FX*=U!!!!!!!!!!!!"
	"!!!!!!!!6%V&Z*MHZ*QHZ*UHZ*UHCXF>!!!!!!!!2$YZG)*?XK!VZK)JZK)JZK1K"
	"ZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+IL@H*<!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RK27[\\5O[\\9O[\\=OU;R1"
	":&V1@X6>P;&?@X6>!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!E72.UTQ>UTY>UT]>"
	"O&]Z!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!EGB0V5Q?"
	"V5U@V5Y@V5Y@V6!?V6!?V6%?VF%?VF)?VF-?VF-?VF1?VF5?<VR/!!!!!!!!!!!!"
	"!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?R'YXW&]>W'!?W'%?W'%?W')?W7-?W71?W71?"
	"W71?W79@W79@K86.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!KHR1X(9@X(=?X(A@X(E@X(I@X8I?X8M?"
	"X8M?X8QAX8UAX8YAX8]AX8]AXI!ANI./!!!!!!!!!!!!!!!!!!!!!!!!2$UZXY9C"
	"XY9CXY=CQ)F+!!!!!!!!!!!!!!!!!!!!!!!!9VF.Y9YEY9YFY9YFQ9^,!!!!!!!!"
	"!!!!!!!!!!!!!!!!9VF/YJ5FYJ9FYJ9FYJ=H!!!!!!!!!!!!!!!!!!!!%2%1X:MV"
	"Z*QHZ*UHZ*UHZ*UHZ*]HZ*]JZ;!JZ;%JZ;%JZK)JZK)JZK1KZK1KZK1K2$YZ!!!!"
	"!!!!!!!!!!!!!!!!R:Z4Z[ELZ[IL[+ILG)6A!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!P*^=[\\5O[\\9O[\\=O[\\=O[\\=O[\\AP[\\EPJ:*D"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!H'6*UTQ>UTY>UT]>Q&IR!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!H7J-V5U@V5Y@V5Y@V6!?"
	"V6!?V6%?VF%?VF)?VF-?VF-?VF1??W*2!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$QY"
	"W&U?W&Y?W&Y?MH&'@'23W'!?W'%?W'%?W')?W7-?W71?W71?W75@W79@K86.!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!)S%BN8Z,X(=?X(A@X(E@X(I@X8I?X8M?X8M?X8QAX8UAX8YA"
	"X8]AX8]ANI..)S%B!!!!!!!!!!!!!!!!!!!!!!!!2$UZXY9CXY9CXY=CW9IQ!!!!"
	"!!!!!!!!!!!!!!!!!!!!=724Y9YEY9YFY9YFY:!F!!!!!!!!!!!!!!!!!!!!!!!!"
	"9VF/YJ5FYJ9FYJ9FYJ=H!!!!!!!!!!!!!!!!!!!!!!!!@G^:Z*QHZ*UHZ*UHZ*UH"
	"Z*]HZ*]JZ;!JZ;%JZ;%JOJ67T:V*ZK1KZK1KZK1K2$YZ!!!!!!!!!!!!!!!!!!!!"
	"R:Z4Z[ELZ[IL[+ILG)6A!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!=GF8[\\5O[\\9O[\\=O[\\=O[\\=O[\\AP[\\EPR[B9!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!NVQYUTQ>UTY>UT]>UU!>!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!9F**Q7-VV5Y@V6!?V6!?V6%?VF%?VF)?"
	"VF-?OGI^6%B#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?OX!`"
	"!!!!9F2+MH.'W'%?W')?W7-?W71?W71?MX6(9F6,!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!='&2N8^-X(E@X(I@X8M?X8M?X8M?X8QAX8UAX8YAS)2!@7J7!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!8&*)XY9CXY9CXY=CXYAC!!!!!!!!!!!!!!!!!!!!"
	"!!!!FHR;Y9YEY9YFY9YFY:!F!!!!!!!!!!!!!!!!!!!!!!!!FXZ<YJ5FYJ9FYJ9F"
	"YJ=H!!!!!!!!!!!!!!!!!!!!!!!!!!!!9VJ/T:F(Z*UHZ*UHZ*]HZ*]JZ;!JT:R*"
	"@H\";!!!!R*J1ZK1KZK1KZK1K4%9`!!!!!!!!!!!!!!!!!!!!Y+AYZ[ELZ[IL[+IL"
	"G)6A!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"J:&C[\\9O[\\=O[\\=O[\\=O[\\AP[\\EPU;V2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!.#]M7UZ'9F.*9F.*9F.*6%B#)S%B!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?OX!`!!!!!!!!!!!!!!!!"
	"6%F$9F2+6%F$!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"%2%12$UY8&&)9V>-9V>-9V>-2$UY)S%B!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!%2%1)S%B)S%B'BE9!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%2%12$][:&V1"
	":&V145>!.$%O!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!2$QYW&U?W&Y?W&Y?OX!`!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!2$QY"
	"W&U?W&Y?W&Y?OX!`!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!9F2+W&U?W&Y?W&Y?OX!`"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!9F2+W&U?W&Y?W&Y?OX!`!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!9F2+W&U?W&Y?W&Y?OX!`!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#1A)"
	")S%B)S%B)S%B'BE9!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!";
