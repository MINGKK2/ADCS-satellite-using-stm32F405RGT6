#include "calibration_store.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>
#include <math.h>
#define JOURNAL_SIZE (128U*1024U)
#define SLOT_SIZE 128U
#define RECORD_MAGIC 0x43414C31UL
#define RECORD_VERSION 1U
#define RECORD_COMMIT 0xC011CA1BUL
#ifdef CALIBRATION_STORE_HOST_TEST
extern uint8_t calibration_test_flash[JOURNAL_SIZE];
#define JOURNAL_BASE ((uintptr_t)calibration_test_flash)
#else
/* STM32F405RG, sector 11; linker FLASH region ends before this address. */
extern uint8_t __calibration_flash_start__,__calibration_flash_end__;
#define JOURNAL_BASE ((uintptr_t)&__calibration_flash_start__)
#endif
typedef struct {
  uint32_t magic,version,sequence,payload_size;
  CalibrationData data;
  uint32_t crc,commit;
} Record;
/* Read the diagonal v2 matrix records written by the experimental firmware.
 * Keep the original v1 API/math and retain the newest gyro/accel calibration.
 * A full non-diagonal/2D-only calibration is not equivalent to the old 3D model.
 */
typedef struct {
  uint32_t layout_tag,flags,measured_flags,mpu_who,mag_type;
  float accel_offset[3],accel_scale[3],mag_offset[3],mag_matrix[9],gyro_bias[3];
} MatrixData;
typedef struct {
  uint32_t magic,version,sequence,payload_size;
  MatrixData data;
  uint32_t crc,commit;
} MatrixRecord;
_Static_assert(sizeof(MatrixRecord)==SLOT_SIZE,"matrix record layout changed");
_Static_assert(sizeof(Record)<=SLOT_SIZE,"calibration record exceeds slot");
_Static_assert(offsetof(Record,commit)+4U==sizeof(Record),"commit must be last word");
static uint32_t crc32(const uint8_t *bytes,size_t size) {
  uint32_t crc=0xFFFFFFFFUL;
  while(size--){crc^=*bytes++;for(unsigned i=0;i<8;i++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);}
  return ~crc;
}
int CalibrationStore_Valid(const CalibrationData *d) {
  if(d->layout_tag!=CALIBRATION_LAYOUT_TAG || !(d->flags&7U) || (d->flags&~7U)
      || (d->measured_flags&~d->flags) || (d->mpu_who!=0x68U && d->mpu_who!=0x70U)
      || d->mag_type>2U)return 0;
  for(unsigned i=0;i<3;i++) {
    if(!isfinite(d->accel_offset[i]) || fabsf(d->accel_offset[i])>32768.0f
        || !isfinite(d->accel_scale[i]) || d->accel_scale[i]<0.5f || d->accel_scale[i]>2.0f
        || !isfinite(d->mag_offset[i]) || fabsf(d->mag_offset[i])>32768.0f
        || !isfinite(d->mag_scale[i]) || d->mag_scale[i]<0.05f || d->mag_scale[i]>20.0f
        || !isfinite(d->gyro_bias[i]) || fabsf(d->gyro_bias[i])>1.0f)return 0;
  }
  return 1;
}
static int decode_record(const Record *r,CalibrationData *data) {
  if(r->magic==RECORD_MAGIC && r->version==2U) {
    const MatrixRecord *m=(const MatrixRecord *)r;
    if(m->payload_size!=sizeof(MatrixData) || m->data.layout_tag!=0xF4053102UL
        || m->commit!=RECORD_COMMIT || (m->data.flags&~7U)
        || m->crc!=crc32((const uint8_t *)m,offsetof(MatrixRecord,crc)))return 0;
    for(unsigned i=0;i<9;i++) {
      if(!isfinite(m->data.mag_matrix[i]))return 0;
      if(i%4U && m->data.mag_matrix[i]!=0.0f)return 0;
    }
    memset(data,0,sizeof(*data));
    data->layout_tag=CALIBRATION_LAYOUT_TAG;data->flags=m->data.flags;
    data->measured_flags=m->data.measured_flags;data->mpu_who=m->data.mpu_who;data->mag_type=m->data.mag_type;
    memcpy(data->accel_offset,m->data.accel_offset,sizeof(data->accel_offset));
    memcpy(data->accel_scale,m->data.accel_scale,sizeof(data->accel_scale));
    memcpy(data->mag_offset,m->data.mag_offset,sizeof(data->mag_offset));
    memcpy(data->gyro_bias,m->data.gyro_bias,sizeof(data->gyro_bias));
    for(unsigned i=0;i<3;i++)data->mag_scale[i]=m->data.mag_matrix[4U*i];
    return CalibrationStore_Valid(data);
  }
  if(!(r->commit==RECORD_COMMIT && r->magic==RECORD_MAGIC && r->version==RECORD_VERSION
      && r->payload_size==sizeof(CalibrationData) && CalibrationStore_Valid(&r->data)
      && r->crc==crc32((const uint8_t *)r,offsetof(Record,crc))))return 0;
  memcpy(data,&r->data,sizeof(*data));return 1;
}
static int valid_record(const Record *r) {
  CalibrationData data;return decode_record(r,&data);
}
static const Record *latest(void) {
  const Record *found=0;
  for(unsigned slot=0;slot<JOURNAL_SIZE/SLOT_SIZE;slot++) {
    const Record *r=(const Record *)(JOURNAL_BASE+slot*SLOT_SIZE);
    if(valid_record(r) && (!found || r->sequence>found->sequence))found=r;
  }
  return found;
}
static int storage_available(void) {
#ifndef CALIBRATION_STORE_HOST_TEST
  if(*(const uint16_t *)FLASHSIZE_BASE<1024U || JOURNAL_BASE!=0x080E0000UL
      || (uintptr_t)&__calibration_flash_end__-JOURNAL_BASE!=JOURNAL_SIZE)return 0;
#endif
  return 1;
}
CalibrationStoreResult CalibrationStore_Load(CalibrationData *data) {
  if(!data || !storage_available())return CAL_STORE_INVALID;
  const Record *r=latest();if(!r)return CAL_STORE_EMPTY;
  return decode_record(r,data)?CAL_STORE_OK:CAL_STORE_INVALID;
}
CalibrationStoreResult CalibrationStore_Save(const CalibrationData *data) {
  if(!data || !storage_available() || !CalibrationStore_Valid(data))return CAL_STORE_INVALID;
  const Record *old=latest();
  CalibrationData previous;
  if(old && decode_record(old,&previous) && !memcmp(&previous,data,sizeof(*data)))return CAL_STORE_OK;
  uintptr_t address=0;
  for(unsigned slot=0;slot<JOURNAL_SIZE/SLOT_SIZE;slot++) {
    const uint32_t *words=(const uint32_t *)(JOURNAL_BASE+slot*SLOT_SIZE);
    unsigned i=0;while(i<SLOT_SIZE/4U && words[i]==0xFFFFFFFFUL)i++;
    if(i==SLOT_SIZE/4U){address=(uintptr_t)words;break;}
  }
  if(!address)return CAL_STORE_FULL; /* never erase stored calibration automatically */
  Record r;memset(&r,0xFF,sizeof(r));r.magic=RECORD_MAGIC;r.version=RECORD_VERSION;
  r.sequence=old?old->sequence+1U:1U;r.payload_size=sizeof(*data);r.data=*data;
  r.crc=crc32((const uint8_t *)&r,offsetof(Record,crc));r.commit=RECORD_COMMIT;
  if(HAL_FLASH_Unlock()!=HAL_OK)return CAL_STORE_IO_ERROR;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR
      |FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
  CalibrationStoreResult result=CAL_STORE_OK;
  for(unsigned offset=0;offset<sizeof(r);offset+=4U) {
    uint32_t word;memcpy(&word,(const uint8_t *)&r+offset,4);
    /* commit word is programmed last, after payload and CRC. */
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address+offset,word)!=HAL_OK){result=CAL_STORE_IO_ERROR;break;}
  }
  (void)HAL_FLASH_Lock();
  /* F405 has an ART data cache for flash reads; invalidate before verify. */
  __HAL_FLASH_DATA_CACHE_DISABLE();__HAL_FLASH_DATA_CACHE_RESET();__HAL_FLASH_DATA_CACHE_ENABLE();
  if(result==CAL_STORE_OK && !valid_record((const Record *)address))result=CAL_STORE_IO_ERROR;
  return result;
}
