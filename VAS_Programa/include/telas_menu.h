// 'BussolaIcon', 16x16px
const unsigned char epd_bitmap_BussolaIcon [] PROGMEM = {
	0x07, 0xe0, 0x18, 0x98, 0x20, 0x9c, 0x50, 0x3a, 0x48, 0x6a, 0x81, 0xd1, 0x82, 0xa1, 0x87, 0xc7, 
	0xe5, 0x41, 0x8b, 0x81, 0x97, 0x01, 0x6c, 0x0a, 0x78, 0x06, 0x30, 0x84, 0x18, 0x98, 0x07, 0xe0
};
// 'GPSIcon', 16x16px
const unsigned char epd_bitmap_GPSIcon [] PROGMEM = {
	0x40, 0x00, 0x20, 0x00, 0xfc, 0x00, 0x87, 0xc0, 0x7b, 0x70, 0x16, 0x10, 0x0c, 0x18, 0x04, 0x34, 
	0x3c, 0x47, 0x23, 0xd8, 0x3c, 0xbb, 0x0c, 0x86, 0x1a, 0x81, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00
};
// 'ItemIco', 16x16px
const unsigned char epd_bitmap_ItemIco [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00, 0x20, 0x70, 0x1f, 0x90, 0x17, 0x90, 0x17, 0x88, 
	0x15, 0x88, 0x17, 0x88, 0x16, 0x08, 0x3f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'BandeiraIcon', 16x16px
const unsigned char epd_bitmap_BandeiraIcon [] PROGMEM = {
	0x00, 0x3c, 0xf8, 0xc3, 0x67, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 
	0x60, 0x3d, 0x78, 0xc3, 0x67, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00
};
// 'CartaoIcon', 16x16px
const unsigned char epd_bitmap_CartaoIcon [] PROGMEM = {
	0x0f, 0xfe, 0x1a, 0xaa, 0x2a, 0xaa, 0x6a, 0xaa, 0xa0, 0x03, 0xa0, 0x03, 0x80, 0x02, 0x40, 0x02, 
	0x80, 0x02, 0x80, 0x02, 0x80, 0x02, 0x80, 0x02, 0x80, 0x02, 0x80, 0x02, 0x80, 0x02, 0xff, 0xfe
};
// 'TurbinaIcon', 16x16px
const unsigned char epd_bitmap_TurbinaIcon [] PROGMEM = {
	0x07, 0xe0, 0x1b, 0x98, 0x22, 0x44, 0x42, 0x22, 0x42, 0x22, 0x82, 0x61, 0x82, 0x41, 0x81, 0x81, 
	0x9e, 0x61, 0xa1, 0x99, 0xc2, 0x85, 0x64, 0x46, 0x58, 0x4a, 0x20, 0x34, 0x18, 0x18, 0x07, 0xe0
};



// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 144)
const int epd_bitmap_allArray_LEN = 6;
const unsigned char* Bitmap_Icons[6] = {
	epd_bitmap_BussolaIcon,
	epd_bitmap_GPSIcon,
	epd_bitmap_ItemIco,
	epd_bitmap_BandeiraIcon,
	epd_bitmap_CartaoIcon,
	epd_bitmap_TurbinaIcon
};



// 'BarraRolagem', 8x64px
const unsigned char epd_bitmap_BarraRolagem [] PROGMEM = {
	0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 
	0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 
	0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 
	0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00
};
// 'BordaSelec', 128x21px
const unsigned char epd_bitmap_BordaSelec [] PROGMEM = {
	0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
	0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00
};

