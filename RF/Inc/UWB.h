#define UWP_Flag_Direction  0x80
#define UWP_Flag_Continue   0x40
#define UWP_Cmd             0xF

#define UWP_Flag_Direction_FromMaster 0x80
#define UWP_Flag_Direction_ToMaster   0

#define UWP_Cmd_DeviceDesc  0x01
#define UWP_Cmd_ChannelDesc 0x02
#define UWP_Cmd_DeviceName  0x03
#define UWP_Cmd_Status      0x04
#define UWP_Cmd_ChangeID    0x05
#define UWP_Cmd_ChangeName  0x06
#define UWP_Cmd_ChangeDescription 0x07
#define UWP_Cmd_Reset       0x0F

#define UWP_Packet_Size     32
uint8_t UWP_Buffer[UWP_Packet_Size];

extern void SendPacket(uint8_t *data,uint8_t size);
void SendCmd(uint8_t cmd);