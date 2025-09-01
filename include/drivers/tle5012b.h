// #ifndef TLE5012B_H
// #define TLE5012B_H

// #include <zephyr/device.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// /* 设备树兼容性字符串 */
// #define DT_DRV_COMPAT infineon_tle5012b

// /* 数据结构 */
// struct tle5012b_data {
// 	int32_t rawdata;
// 	int32_t covdata;
// 	int32_t filterdata;
// 	int16_t column;
// };

// /* 函数声明 */
// int tle5012b_get_data(const struct device *dev, struct tle5012b_data *data);
// int tle5012b_init(const struct device *dev);

// #ifdef __cplusplus
// }
// #endif

// #endif /* TLE5012B_H */