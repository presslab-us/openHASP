/* MIT License - Copyright (c) 2019-2024 Francis Van Roie
 For full license information read the LICENSE file in the project folder */

#ifndef HASP_SDL2_DRIVER_H
#define HASP_SDL2_DRIVER_H

#include "tft_driver.h"

#if USE_MONITOR && HASP_TARGET_PC
// #warning Building H driver TFT SDL2

#include "lvgl.h"
#include "indev/mouse.h"

namespace dev {

class TftSdl : BaseTft {
  public:
    void init(int w, int h);
    void show_info();
    void splashscreen();

    void set_rotation(uint8_t rotation);
    void set_invert(bool invert);

    void flush_pixels(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p);
    bool is_driver_pin(uint8_t pin);

    const char* get_tft_model();

    int32_t width();
    int32_t height();

  private:
    int32_t _width, _height;
};

} // namespace dev

using dev::TftSdl;
extern dev::TftSdl haspTft;

#endif // HASP_TARGET_PC

#endif // HASP_SDL2_DRIVER_H
