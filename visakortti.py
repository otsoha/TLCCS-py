import usb.core
dev = usb.core.find(idVendor=0x1313, idProduct=0x8087)
assert dev is not None
dev.set_configuration()
cfg = dev.get_active_configuration()
print(cfg)