/* Definitions for WIZNET Ethernet W5100 chip */

/* ****** Common Registers ****** */

/* Mode */
#define MR 0x0000

/* Mode Register Bits */
#define MD_IND 0 //Indirect Bus I/F Mode
#define MD_AI 1 //Address Auto-Increment in Indirect Bus I/F
#define MD_LB 2 //Not User (Little/Big Endian Selector in I/F Mode)
#define MD_PPPoE 3 //PPPoE Mode
#define MR_PB 4 //Ping Block Mode
#define MR_RST 7 //S/W Reset

/* Gateway Address */
#define GAR0 0x0001
#define GAR1 0x0002
#define GAR2 0x0003
#define GAR3 0x0004

/* Subnet Mask Address */
#define SUBR0 0x0005
#define SUBR1 0x0006
#define SUBR2 0x0007
#define SUBR3 0x0008

/* Source Hardware Address */
#define SHAR0 0x0009
#define SHAR1 0x000A
#define SHAR2 0x000B
#define SHAR3 0x000C
#define SHAR4 0x000D
#define SHAR5 0x000E

/* Source IP Address */
#define SIPR0 0x000F
#define SIPR1 0x0010
#define SIPR2 0x0011
#define SIPR3 0x0012

//0x0013-0x0014 Reserved

/* Interrupt */
#define IR 0x0015

/* Interrupt Register Bits */
#define IR_S0_INT 0 //Occurrence of Socket 0 Socket Interrupt
#define IR_S1_INT 1 //Occurrence of Socket 1 Socket Interrupt
#define IR_S2_INT 2 //Occurrence of Socket 2 Socket Interrupt
#define IR_S3_INT 3 //Occurrence of Socket 3 Socket Interrupt
#define IR_PPPoE 5 //PPPoE Connection Close
#define IR_UNREACH 6 //Destination unreachable
#define IR_CONFLICT 7 //IP Conflict

/* Interrupt Mask */
#define IMR 0x0016

/* Interrupt Mask Register Bits */
#define IM_IR0 0 //Occurrence of Socket 0 Socket Interrupt Enable
#define IM_IR1 1 //Occurrence of Socket 1 Socket Interrupt Enable 
#define IM_IR2 2 //Occurrence of Socket 2 Socket Interrupt Enable 
#define IM_IR3 3 //Occurrence of Socket 3 Socket Interrupt Enable 
#define IM_IR5 5 //PPPoE Close Enable 
#define IM_IR6 6 //Destination unreachable Enable 
#define IM_IR7 7 //IP Conflict Enable 

/* Retry Time */
#define RTR0 0x0017
#define RTR1 0x0018

/* Retry Count */
#define RCR 0x0019

/* RX Memory Size */
#define RMSR 0x001A

/* TX Memory Size */
#define TMSR 0x001B

/* Authentication Type in PPPoE */
#define PATR0 0x001C
#define PATR1 0x001D

//0x001E-0x0027 Reserved

/* PPP LCP Request Timer */
#define PTIMER 0x0028

/* PPP LCP Magic Number */
#define PMAGIC 0x0029

/* Unreachable IP Address */
#define UIPR0 0x002A
#define UIPR1 0x002B
#define UIPR2 0x002C
#define UIPR3 0x002D

/* Unreachable Port */
#define UPORT0 0x002E
#define UPORT1 0x002F

//0x0030-0x03FF Reserved

/* ****** Socket Registers ****** */

/* ** General Definitions ** */

/* Socket Mode Register Bits*/

/* Protocol Select */
/* P3 P2 P1 P0 Meaning 
   0  0  0  0  Closed 
   0  0  0  1  TCP 
   0  0  1  0  UDP 
   0  0  1  1  IPRAW */

/* Protocol Select Socket 0 Only */
/* P3 P2 P1 P0 Meaning 
   0  1  0  0  MACRAW 
   0  1  0  1  PPPoE */

#define Sn_MR_P0 0 //Protocol Select
#define Sn_MR_P1 1 //Protocol Select
#define Sn_MR_P2 2 //Protocol Select
#define Sn_MR_P3 3 //Protocol Select
#define Sn_MR_ND_MC //No Delayed ACK | Mutlicast IGMP version
#define Sn_MR_MULTI //Multicast Enable for UDP

/* Socket Command Register Commands */
#define Sn_CR_OPEN 0x01 //Open Socket
#define Sn_CR_LISTEN 0x02 //Listen in TCP mode
#define Sn_CR_CONNECT 0x04 //Connect in TCP mode
#define Sn_CR_DISCONN 0x08 //Disconnect in TCP mode
#define Sn_CR_CLOSE 0x10 //Close Socket
#define Sn_CR_SEND 0x20 //Transmit Data from TX Register
#define Sn_CR_SEND_MAC 0x21 //Send to MAC Address in UDP mode
#define Sn_CR_SEND_KEEP 0x22 //Keep Connection Alive in TCP mode
#define Sn_CR_RECV 0x40 //Receive Data into RX Register

/* Socket Interrupt Register Bits */
#define Sn_IR_CON 0 //Connection Established
#define Sn_IR_DISCON 1 //Connection Terminated or Finished
#define Sn_IR_RECV 2 //Data is Received
#define Sn_IR_TIMEOUT //Timout Occured
#define Sn_IR_SEND_OK //Data is Sent

/* Socket Status Register Values */
//Fixed Status
#define Sn_SR_SOCK_CLOSED 0x00
#define Sn_SR_SOCK_INIT 0x13
#define Sn_SR_SOCK_LISTEN 0x14
#define Sn_SR_SOCK_ESTABLISHED 0x17
#define Sn_SR_SOCK_CLOSE_WAIT 0x1C
#define Sn_SR_SOCK_UDP 0x22
#define Sn_SR_SOCK_IPRAW 0x32
#define Sn_SR_SOCK_MACRAW 0x42
#define Sn_SR_SOCK_PPPOE 0x5F
//Changing Status
#define Sn_SR_SOCK_SYNSENT 0x15
#define Sn_SR_SOCK_SYNRECV 0x16
#define Sn_SR_SOCK_FIN_WAIT 0x18
#define Sn_SR_SOCK_CLOSING 0x1A
#define Sn_SR_SOCK_TIME_WAIT 0x1B
#define Sn_SR_SOCK_LAST_ACK 0x1D
#define Sn_SR_SOCK_ARP0 0x11
#define Sn_SR_SOCK_ARP1 0x21
#define Sn_SR_SOCK_ARP2 0x31

/* ** Socket 0 ** */

/* Socket 0 Mode */
#define S0_MR 0x0400

/* Socket 0 Command */
#define S0_CR 0x0401

/* Socket 0 Interrupt */
#define S0_IR 0x0402

/* Socket 0 Status */
#define S0_SR 0x0403

/* Socket 0 Source Port */
#define S0_PORT0 0x0404
#define S0_PORT1 0x0405

/* Socket 0 Destination Hardware Address */
#define S0_DHAR0 0x0406
#define S0_DHAR1 0x0407
#define S0_DHAR2 0x0408
#define S0_DHAR3 0x0409
#define S0_DHAR4 0x040A
#define S0_DHAR5 0x040B

/* Socket 0 Destination IP Address */
#define S0_DIPR0 0x040C
#define S0_DIPR1 0x040D
#define S0_DIPR2 0x040E
#define S0_DIPR3 0x040F

/* Socket 0 Destination Port */
#define S0_DPORT0 0x0410
#define S0_DPORT1 0x0411

/* Socket 0 Maximum Segment Size */
#define S0_MSSR0 0x0412
#define S0_MSSR1 0x0413

/* Socket 0 Protocol in IP Raw mode */
#define S0_PROTO 0x0414

/* Socket 0 IP TOS */
#define S0_TOS 0x0415

/* Socket 0 IP TTL */
#define S0_TTL 0x0416

//0x0417-0x041F Reserved

/* Socket 0 TX Free Size */
#define S0_TX_FSR0 0x0420
#define S0_TX_FSR1 0x0421

/* Socket 0 TX Read Pointer */
#define S0_TX_RD0 0x0422
#define S0_TX_RD1 0x0423

/* Socket 0 TX Write Pointer */
#define S0_TX_WR0 0x0424
#define S0_TX_WR1 0x0425

/* Socket 0 RX Received Size */
#define S0_RX_RSR0 0x0426
#define S0_RX_RSR1 0x0427

/* Socket 0 RX Read Pointer */
#define S0_RX_RD0 0x0428
#define S0_RX_RD1 0x0429

//0x042A-0x042B Reserved

//0x042C-0x04FF Reserved

/* ** Socket 1 ** */

/* Socket 1 Mode */
#define S1_MR 0x0500

/* Socket 1 Command */
#define S1_CR 0x0501

/* Socket 1 Interrupt */
#define S1_IR 0x0502

/* Socket 1 Status */
#define S1_SR 0x0503

/* Socket 1 Source Port */
#define S1_PORT0 0x0504
#define S1_PORT1 0x0505

/* Socket 1 Destination Hardware Address */
#define S1_DHAR0 0x0506
#define S1_DHAR1 0x0507
#define S1_DHAR2 0x0508
#define S1_DHAR3 0x0509
#define S1_DHAR4 0x050A
#define S1_DHAR5 0x050B

/* Socket 1 Destination IP Address */
#define S1_DIPR0 0x050C
#define S1_DIPR1 0x050D
#define S1_DIPR2 0x050E
#define S1_DIPR3 0x050F

/* Socket 1 Destination Port */
#define S1_DPORT0 0x0510
#define S1_DPORT1 0x0511

/* Socket 1 Maximum Segment Size */
#define S1_MSSR0 0x0512
#define S1_MSSR1 0x0513

/* Socket 1 Protocol in IP Raw mode */
#define S1_PROTO 0x0514

/* Socket 1 IP TOS */
#define S1_TOS 0x0515

/* Socket 1 IP TTL */
#define S1_TTL 0x0516

//0x0517-0x051F Reserved

/* Socket 1 TX Free Size */
#define S1_TX_FSR0 0x0520
#define S1_TX_FSR1 0x0521

/* Socket 1 TX Read Pointer */
#define S1_TX_RD0 0x0522
#define S1_TX_RD1 0x0523

/* Socket 1 TX Write Pointer */
#define S1_TX_WR0 0x0524
#define S1_TX_WR1 0x0525

/* Socket 1 RX Received Size */
#define S1_RX_RSR0 0x0526
#define S1_RX_RSR1 0x0527

/* Socket 1 RX Read Pointer */
#define S1_RX_RD0 0x0528
#define S1_RX_RD1 0x0529

//0x052A-0x052B Reserved

//0x052C-0x05FF Reserved

/* ** Socket 2 ** */

/* Socket 2 Mode */
#define S2_MR 0x0600

/* Socket 2 Command */
#define S2_CR 0x0601

/* Socket 2 Interrupt */
#define S2_IR 0x0602

/* Socket 2 Status */
#define S2_SR 0x0603

/* Socket 2 Source Port */
#define S2_PORT0 0x0604
#define S2_PORT1 0x0605

/* Socket 2 Destination Hardware Address */
#define S2_DHAR0 0x0606
#define S2_DHAR1 0x0607
#define S2_DHAR2 0x0608
#define S2_DHAR3 0x0609
#define S2_DHAR4 0x060A
#define S2_DHAR5 0x060B

/* Socket 2 Destination IP Address */
#define S2_DIPR0 0x060C
#define S2_DIPR1 0x060D
#define S2_DIPR2 0x060E
#define S2_DIPR3 0x060F

/* Socket 2 Destination Port */
#define S2_DPORT0 0x0610
#define S2_DPORT1 0x0611

/* Socket 2 Maximum Segment Size */
#define S2_MSSR0 0x0612
#define S2_MSSR1 0x0613

/* Socket 2 Protocol in IP Raw mode */
#define S2_PROTO 0x0614

/* Socket 2 IP TOS */
#define S2_TOS 0x0615

/* Socket 2 IP TTL */
#define S2_TTL 0x0616

//0x0617-0x061F Reserved

/* Socket 2 TX Free Size */
#define S2_TX_FSR0 0x0620
#define S2_TX_FSR1 0x0621

/* Socket 2 TX Read Pointer */
#define S2_TX_RD0 0x0622
#define S2_TX_RD1 0x0623

/* Socket 2 TX Write Pointer */
#define S2_TX_WR0 0x0624
#define S2_TX_WR1 0x0625

/* Socket 2 RX Received Size */
#define S2_RX_RSR0 0x0626
#define S2_RX_RSR1 0x0627

/* Socket 2 RX Read Pointer */
#define S2_RX_RD0 0x0628
#define S2_RX_RD1 0x0629

//0x062A-0x062B Reserved

//0x062C-0x06FF Reserved

/* ** Socket 3 ** */

/* Socket 3 Mode */
#define S3_MR 0x0700

/* Socket 3 Command */
#define S3_CR 0x0701

/* Socket 3 Interrupt */
#define S3_IR 0x0702

/* Socket 3 Status */
#define S3_SR 0x0703

/* Socket 3 Source Port */
#define S3_PORT0 0x0704
#define S3_PORT1 0x0705

/* Socket 3 Destination Hardware Address */
#define S3_DHAR0 0x0706
#define S3_DHAR1 0x0707
#define S3_DHAR2 0x0708
#define S3_DHAR3 0x0709
#define S3_DHAR4 0x070A
#define S3_DHAR5 0x070B

/* Socket 3 Destination IP Address */
#define S3_DIPR0 0x070C
#define S3_DIPR1 0x070D
#define S3_DIPR2 0x070E
#define S3_DIPR3 0x070F

/* Socket 3 Destination Port */
#define S3_DPORT0 0x0710
#define S3_DPORT1 0x0711

/* Socket 3 Maximum Segment Size */
#define S3_MSSR0 0x0712
#define S3_MSSR1 0x0713

/* Socket 3 Protocol in IP Raw mode */
#define S3_PROTO 0x0714

/* Socket 3 IP TOS */
#define S3_TOS 0x0715

/* Socket 3 IP TTL */
#define S3_TTL 0x0716

//0x0717-0x071F Reserved

/* Socket 3 TX Free Size */
#define S3_TX_FSR0 0x0720
#define S3_TX_FSR1 0x0721

/* Socket 3 TX Read Pointer */
#define S3_TX_RD0 0x0722
#define S3_TX_RD1 0x0723

/* Socket 3 TX Write Pointer */
#define S3_TX_WR0 0x0724
#define S3_TX_WR1 0x0725

/* Socket 3 RX Received Size */
#define S3_RX_RSR0 0x0726
#define S3_RX_RSR1 0x0727

/* Socket 3 RX Read Pointer */
#define S3_RX_RD0 0x0728
#define S3_RX_RD1 0x0729

//0x072A-0x072B Reserved

//0x072C-0x07FF Reserved

/* ****** Reserved Registers ****** */
/*     0x0800 to 0x3FFF Reserved    */

/* ****** TX Registers ****** */
/* 0x4000 to 0x5FFF TX Memory */

#define TX_BASE 0x4000

/* ****** RX Registers ****** */
/* 0x6000 to 0x7FFF RX Memory */

#define RX_BASE 0x6000
