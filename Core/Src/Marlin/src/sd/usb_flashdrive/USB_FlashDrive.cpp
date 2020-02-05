#include "USB_FlashDrive.h"
#include "usb_host.h"
#include "usbh_msc.h"

#if  ENABLED(USB_MASS_STORAGE_SUPPORT)
extern USBH_HandleTypeDef hUsbHostFS;

bool Sd2Card::init(const uint8_t sckRateID, const pin_t chipSelectPin)
{
	if (!isInserted())
	{
		return false;
	}

	return true;
}

bool Sd2Card::readBlock(uint32_t block, uint8_t *dst)
{
	if (USBH_MSC_Read(&hUsbHostFS, 0, block, dst, 1) == USBH_OK)
	{
		return true;
	}
	return false;
}

bool Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t *src)
{
	if (USBH_MSC_Write(&hUsbHostFS, 0, blockNumber, (uint8_t *)src, 1) == USBH_OK)
	{
		return true;
	}
	return false;
}

uint32_t Sd2Card::cardSize()
{
	return 0;
}

bool Sd2Card::isInserted()
{
	return USB_HostAppStatus() == APPLICATION_READY;
}

bool Sd2Card::ready()
{//queue.inject_P(M21_STR);
	return isInserted();
}

#endif
