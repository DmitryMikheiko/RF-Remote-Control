#include "UWB.h"
void SendCmd(u8 cmd)
{
	SendPacket(cmd | UWP_Flag_Direction_FromMaster);
}
void SendData(u8 cmd,u8 *data,u8 size)
{
	u8 p=2,dp=0;
	UWP_Buffer[0]=cmd | UWP_Flag_Direction_FromMaster;
	UWP_Buffer[1]=size;
	while(dp<size)
	{
	  while(p<UWP_Packet_Size)
	  {
		  UWP_Buffer[p++]=data[dp++];
	  }
		SendPacket(UWP_Buffer,p);
		UWP_Buffer[0]|=UWP_Flag_Continue;
		p=1;
  }
}
void ProccPacket(u8 *data,u8 *size)
{
	
}