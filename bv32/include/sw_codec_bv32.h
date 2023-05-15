#ifndef __SW_CODEC_BV32_H__
#define __SW_CODEC_BV32_H__

#include <zephyr/types.h>

#define LC3_USE_BITRATE_FROM_INIT 0

/*! \addtogroup LC3_translation
 *  \{ */

/**@brief	Runs the LC3 encoder
 *
 * @param[in] pcm_data                  Buffer containing PCM data
 * @param[in] pcm_data_size             Number of bytes in pcm_data. Note that
 *                                      pcm_data is uint16_t *.
 * @param[in] enc_bitrate		Bitrate of encoded mono stream (bps).
 *					If set to LC3_USE_BITRATE_FROM_INIT
 *					the bitrate given to
 *					sw_codec_lc3_enc_init() will be used.
 *					Consult the LC3 doc for legal values.
 * @param[in] audio_ch                  Index to which channel is being encoded
 * @param[in] lc3_data_buf_size         Size of supplied LC3 buffer.
 *
 * @param[out] lc3_data                 Pointer to LC3 data output buffer.
 * @param[out] lc3_data_wr_size         Number of bytes written to lc3_data.
 *
 *
 * @note	The return codes from Zephyr and LC3 may overlap.
 *
 * @return 0 on success.	-EPERM Encoder is not initialized.
 *				-EINVAL Too few PCM bytes given to encode.
 *				Other errors. Refer to LC3 documentation.
 */
int sw_codec_lc3_enc_run(void const *const pcm_data, uint32_t pcm_data_size, uint32_t enc_bitrate,
			 uint8_t audio_ch, uint16_t lc3_data_buf_size, uint8_t *const lc3_data,
			 uint16_t *const lc3_data_wr_size);

/**@brief	Runs the LC3 decoder
 *
 * @param[in] lc3_data                  Buffer containing LC3 data.
 * @param[in] lc3_data_size             Number of bytes in lc3_data.
 * @param[in] pcm_data_buf_size         Size of supplied pcm_data buffer
 * @param[in] audio_ch  		Index to which channel is being decoded
 *
 * @param[out] pcm_data                 Pointer to PCM data output buffer.
 * @param[out] pcm_data_wr_size		Number of bytes written to pcm_data
 * @param[out] bad_frame                Bad frame indicator. Refer to LC3 doc.
 *
 *
 * @note	The return codes from Zephyr and LC3 may overlap.
 *
 * @return 0 on success.	-EPERM Decoder is not initialized.
 *				Other errors. Refer to LC3 documentation.
 */
int sw_codec_lc3_dec_run(uint8_t const *const lc3_data, uint16_t lc3_data_size,
			 uint16_t pcm_data_buf_size, uint8_t audio_ch, void *const pcm_data,
			 uint16_t *const pcm_data_wr_size, bool bad_frame);

/**@brief	Closes the LC3 encoder and frees allocated RAM
 *
 * @return 0 on success.	-EALREADY if already un-initialized.
 */
int sw_codec_lc3_enc_uninit_all(void);

/**@brief	Closes the LC3 decoder and frees allocated RAM
 *
 *  @return 0 on success.	-EALREADY if already un-initialized.
 */
int sw_codec_lc3_dec_uninit_all(void);

/**@brief Initializes the LC3 Codec
 *
 * @note: For documentation, see LC3API.h (/codec/inc/LC3API.h)
 */
int sw_codec_lc3_init(uint8_t *sw_codec_lc3_buffer, uint32_t *sw_codec_lc3_buffer_size,
		      uint16_t framesize_us);

/**@brief	Initializes the LC3 encoder and allocates required RAM
 *
 * @param[in]	pcm_sample_rate		Sample rate in Hz (typ. 16000 or 48000)
 * @param[in]	pcm_bit_depth		Number of bits in sample (typ. 16 or 24)
 * @param[in]	framesize_us		Frame size in microseconds
 * @param[in]	enc_bitrate		Bitrate of encoded mono stream (bps)
 *					(typ. 24000 - 160000). Consult the LC3
 *					doc for legal values.
 * @param[in]   num_channels		Number of channels to initialize
 *
 * @param[out]	pcm_bytes_req		PCM bytes required to encode a frame.
 *
 * @note	The return codes from Zephyr and LC3 may overlap.
 *
 * @return 0 on success.	-EALREADY if already initialized.
 *				-EPERM 0 bytes required from PCM sample.
 *				Other errors. Refer to LC3 documentation.
 */
int sw_codec_lc3_enc_init(uint16_t pcm_sample_rate, uint8_t pcm_bit_depth, uint16_t framesize_us,
			  uint32_t enc_bitrate, uint8_t num_channels,
			  uint16_t *const pcm_bytes_req);

/**@brief 	Initializes the LC3 decoder and allocates required RAM
 *
 * @param[in]	pcm_sample_rate		Sample rate in Hz (typ. 16000 or 48000)
 * @param[in]	pcm_bit_depth		Output PCM bits per sample.
 * @param[in]	framesize_us		Frame size in microseconds
 * @param[in]   num_channels		Number of channels to initialize
 *
 * @note	The return codes from Zephyr and LC3 may overlap.
 *
 * @return 0 on success.	-EALREADY if already initialized.
 *				Other errors. Refer to LC3 documentation.
 */
int sw_codec_lc3_dec_init(uint16_t pcm_sample_rate, uint8_t pcm_bit_depth, uint16_t framesize_us,
			  uint8_t num_channels);

/*! \} */    /* LC3_translation */

#endif /* __SW_CODEC_LC3_H__ */
