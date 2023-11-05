#include <M5Unified.h>
#include <esp_camera.h>

#include <fb_gfx.h>
#include <human_face_detect_msr01.hpp>
#include <human_face_detect_mnp01.hpp>

#include <SD.h>

#define TWO_STAGE 1 /*<! 1: detect by two-stage which is more accurate but slower(with keypoints). */
                    /*<! 0: detect by one-stage which is less accurate but faster(without keypoints). */

#define LCD_WIDTH (320)
#define LCD_HEIGHT (240)
#define LCD_BUF_SIZE (LCD_WIDTH * LCD_HEIGHT * 2)

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 2
#define SIOD_GPIO_NUM 12
#define SIOC_GPIO_NUM 11
#define Y9_GPIO_NUM 47
#define Y8_GPIO_NUM 48
#define Y7_GPIO_NUM 16
#define Y6_GPIO_NUM 15
#define Y5_GPIO_NUM 42
#define Y4_GPIO_NUM 41
#define Y3_GPIO_NUM 40
#define Y2_GPIO_NUM 39
#define VSYNC_GPIO_NUM 46
#define HREF_GPIO_NUM 38
#define PCLK_GPIO_NUM 45

#define SD_CS 4
#define SPI_MOSI 37
#define SPI_MISO 35
#define SPI_SCK 36
#define PATHNAME_SIZE 255
char pathname[PATHNAME_SIZE];

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565, // PIXFORMAT_RGB565 for face detection/recognition, PIXFORMAT_JPEG for streaming
    .frame_size = FRAMESIZE_QVGA,     // QVGA(320x240), FRAMESIZE_UXGA
    .jpeg_quality = 0,                // 12
    .fb_count = 2,                    // 1
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

esp_err_t camera_init()
{
  // initialize the camera
  M5.In_I2C.release();
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return err;
  }

  return ESP_OK;
}

void check_heap_free_size(void)
{
  Serial.printf("===============================================================\n");
  Serial.printf("Mem Test\n");
  Serial.printf("===============================================================\n");
  Serial.printf("esp_get_free_heap_size()                              : %6d\n", esp_get_free_heap_size());
  Serial.printf("heap_caps_get_free_size(MALLOC_CAP_DMA)               : %6d\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
  Serial.printf("heap_caps_get_free_size(MALLOC_CAP_SPIRAM)            : %6d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  Serial.printf("heap_caps_get_free_size(MALLOC_CAP_INTERNAL)          : %6d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  Serial.printf("heap_caps_get_free_size(MALLOC_CAP_DEFAULT)           : %6d\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
}

#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

static void draw_face_boxes(fb_data_t *fb, std::list<dl::detect::result_t> *results, int face_id)
{
  int x, y, w, h;
  uint32_t color = FACE_COLOR_YELLOW;
  if (face_id < 0)
  {
    color = FACE_COLOR_RED;
  }
  else if (face_id > 0)
  {
    color = FACE_COLOR_GREEN;
  }
  if (fb->bytes_per_pixel == 2)
  {
    // color = ((color >> 8) & 0xF800) | ((color >> 3) & 0x07E0) | (color & 0x001F);
    color = ((color >> 16) & 0x001F) | ((color >> 3) & 0x07E0) | ((color << 8) & 0xF800); // BGR888 -> RGB565
  }
  int i = 0;
  for (std::list<dl::detect::result_t>::iterator prediction = results->begin(); prediction != results->end(); prediction++, i++)
  {
    // rectangle box
    x = (int)prediction->box[0];
    y = (int)prediction->box[1];

    // yが負の数のときにfb_gfx_drawFastHLine()でメモリ破壊してリセットする不具合の対策
    if (y < 0)
    {
      y = 0;
    }

    w = (int)prediction->box[2] - x + 1;
    h = (int)prediction->box[3] - y + 1;

    // Serial.printf("x:%d y:%d w:%d h:%d\n", x, y, w, h);

    if ((x + w) > fb->width)
    {
      w = fb->width - x;
    }
    if ((y + h) > fb->height)
    {
      h = fb->height - y;
    }

    // Serial.printf("x:%d y:%d w:%d h:%d\n", x, y, w, h);

    // fb_gfx_fillRect(fb, x+10, y+10, w-20, h-20, FACE_COLOR_RED);  //モザイク
    fb_gfx_drawFastHLine(fb, x, y, w, color);
    fb_gfx_drawFastHLine(fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(fb, x, y, h, color);
    fb_gfx_drawFastVLine(fb, x + w - 1, y, h, color);

#if TWO_STAGE
    // landmarks (left eye, mouth left, nose, right eye, mouth right)
    int x0, y0, j;
    for (j = 0; j < 10; j += 2)
    {
      x0 = (int)prediction->keypoint[j];
      y0 = (int)prediction->keypoint[j + 1];
      fb_gfx_fillRect(fb, x0, y0, 3, 3, color);
    }
#endif
  }
}

static std::list<dl::detect::result_t> face_detect(camera_fb_t *fb)
{

#if TWO_STAGE
  // box推論モデル準備(上位推定箇所10個) 解像度を落として、顔とおもわしき位置を予測
  HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
  /**
   * @brief Construct a new Human Face Detect MSR01 object
   *
   * @param score_threshold   predicted boxes with score lower than the threshold will be filtered out
   * @param nms_threshold     predicted boxes with IoU higher than the threshold will be filtered out
   * @param top_k             first k highest score boxes will be remained
   * @param resize_scale      resize scale to implement on input image
   */

  // 細部推論モデル準備(上位推定箇所5個)
  HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
  /**
   * @brief Construct a new Human Face Detect MNP01 object.
   *
   * @param score_threshold predicted boxes with score lower than the threshold will be filtered out
   * @param nms_threshold   predicted boxes with IoU higher than the threshold will be filtered out
   * @param top_k           first k highest score boxes will be remained
   */

  // 推論ステージ1
  std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
  /**
   * @brief Inference.
   *
   * @tparam T supports uint8_t(3byte) and uint16_t(2byte)
   *         - uint8_t: input image is RGB888
   *         - uint16_t: input image is RGB565
   * @param input_element pointer of input image
   * @param input_shape   shape of input image {高さ,幅,RGBの3要素}
   * @return detection result
   */

  // 推論ステージ2
  std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3}, candidates);
  /**
   * @brief Inference.
   *
   * @tparam T supports uint16_t and uint8_t,
   *         - uint16_t: input image is RGB565
   *         - uint8_t: input image is RGB888
   * @param input_element pointer of input image
   * @param input_shape   shape of input image
   * @param candidates    candidate boxes on input image (候補ボックス)
   * @return detection result
   */
#else
  HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
  std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
#endif

  return results;
}

void setup()
{
  M5.begin();

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  while (false == SD.begin(SD_CS, SPI))
  {
    delay(100);
  }

  camera_init();

  auto spk_cfg = M5.Speaker.config();
  M5.Speaker.config(spk_cfg);
  M5.Speaker.begin();
  if (!M5.Speaker.isEnabled())
  {
    M5_LOGE("Speaker not found...");
  }
  M5.Speaker.tone(2000, 100);

  // M5.Display.setFont(&fonts::efontJA_24);
}

void loop()
{
  M5.update();

  M5.In_I2C.release();
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    M5_LOGE("Camera Capture Failed");
    return;
  }

  // snap shot to sd card
  auto touch_detail = M5.Touch.getDetail();
  if (M5.BtnA.wasPressed() | touch_detail.wasClicked())
  {
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;

    for (int i = 0; i < 3; i++)
    {
      M5.Speaker.tone(2000, 100);
      delay(1000);
    }
    if (fb->format != PIXFORMAT_JPEG)
    {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      if (!jpeg_converted)
      {
        M5_LOGE("JPEG compression failed");
        return;
      }
    }
    auto dt = M5.Rtc.getDateTime();
    snprintf(pathname, PATHNAME_SIZE, "/IMG_%04d%02d%02d_%02d%02d%02d.JPG",
             dt.date.year, dt.date.month, dt.date.date,
             dt.time.hours, dt.time.minutes, dt.time.seconds);
    File file = SD.open(pathname, FILE_WRITE);
    file.write(_jpg_buf, _jpg_buf_len);
    file.close();
    Serial.printf("saved %s\n", pathname);
  }

  std::list<dl::detect::result_t> results = face_detect(fb);
  if (results.size() > 0)
  {
    Serial.printf("Face detected : %d\n", results.size());

    fb_data_t rfb;
    rfb.width = fb->width;
    rfb.height = fb->height;
    rfb.bytes_per_pixel = 2;
    rfb.format = FB_RGB565;
    rfb.data = fb->buf;
    draw_face_boxes(&rfb, &results, 0);
  }

  M5.Display.startWrite();
  M5.Display.setAddrWindow(0, 0, LCD_WIDTH, LCD_HEIGHT);
  M5.Display.writePixels((uint16_t *)fb->buf, int(fb->len / 2));
  M5.Display.endWrite();

  esp_camera_fb_return(fb);

  // if (results.size() > 0)
  // {
  //   delay(1000 * 5); // 5 seconds
  //   M5.Power.deepSleep(5 * 60 * pow(10, 6)); // 5 minutes
  // }
}