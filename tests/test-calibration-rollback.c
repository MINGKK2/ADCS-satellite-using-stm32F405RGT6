#include "calibration_store.h"
#include "axis_frame.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
_Alignas(4) uint8_t calibration_test_flash[128U*1024U];
static int fail_after=-1,programs;
HAL_StatusTypeDef HAL_FLASH_Unlock(void){return HAL_OK;}
HAL_StatusTypeDef HAL_FLASH_Lock(void){return HAL_OK;}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type,uintptr_t address,uint64_t data){
  (void)type;assert(address>=(uintptr_t)calibration_test_flash);
  assert(address+4<=(uintptr_t)calibration_test_flash+sizeof(calibration_test_flash));
  if(fail_after==0)return HAL_ERROR;
  if(fail_after>0)fail_after--;
  uint32_t old;memcpy(&old,(void *)address,4);uint32_t word=(uint32_t)data;
  assert((old&word)==word);memcpy((void *)address,&word,4);programs++;return HAL_OK;
}
static CalibrationData fixture(void){
  CalibrationData d={0};d.layout_tag=CALIBRATION_LAYOUT_TAG;d.flags=d.measured_flags=7;
  d.mpu_who=0x70;d.mag_type=2;
  for(unsigned i=0;i<3;i++){d.accel_offset[i]=(float)i*200;d.accel_scale[i]=1.05f;d.mag_offset[i]=-(float)i*150;d.mag_scale[i]=.99f;d.gyro_bias[i]=.001f*(float)i;}
  return d;
}
static uint32_t crc32(const void *ptr,size_t size){
  const uint8_t *p=ptr;uint32_t c=~0U;
  while(size--){c^=*p++;for(unsigned j=0;j<8;j++)c=(c>>1)^((c&1)?0xEDB88320U:0);}
  return ~c;
}
typedef struct {
  uint32_t layout_tag,flags,measured_flags,who,type;
  float ao[3],as[3],mo[3],matrix[9],gb[3];
} MatrixData;
typedef struct {uint32_t magic,version,sequence,size;MatrixData data;uint32_t crc,commit;} MatrixRecord;
static void commit_matrix(MatrixRecord *m){m->crc=crc32(m,offsetof(MatrixRecord,crc));memcpy(calibration_test_flash+128,m,sizeof(*m));}
int main(int argc,char **argv){
  CalibrationData d=fixture(),out;
  memset(calibration_test_flash,255,sizeof(calibration_test_flash));
  assert(CalibrationStore_Load(&out)==CAL_STORE_EMPTY);
  assert(CalibrationStore_Save(&d)==CAL_STORE_OK);
  assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&!memcmp(&d,&out,sizeof(d)));
  int saved=programs;assert(CalibrationStore_Save(&d)==CAL_STORE_OK&&programs==saved);
  /* New operational axes do not change the historical Flash bias encoding. */
  float body_bias[3];ADCS_GyroBiasLoad(out.gyro_bias,body_bias);
  assert(body_bias[0]==out.gyro_bias[1]&&body_bias[1]==-out.gyro_bias[0]&&body_bias[2]==out.gyro_bias[2]);
  CalibrationData roundtrip=out;ADCS_GyroBiasSave(body_bias,roundtrip.gyro_bias);
  assert(!memcmp(&roundtrip,&out,sizeof(out)));
  assert(CalibrationStore_Save(&roundtrip)==CAL_STORE_OK&&programs==saved);
  puts("PASS changed body axes preserve legacy Flash calibration bytes and journal dedup");
  CalibrationData bad=d;bad.mag_scale[0]=NAN;assert(CalibrationStore_Save(&bad)==CAL_STORE_INVALID);
  bad=d;bad.layout_tag++;assert(CalibrationStore_Save(&bad)==CAL_STORE_INVALID);
  for(int failure=0;failure<26;failure++){
    memset(calibration_test_flash,255,sizeof(calibration_test_flash));fail_after=-1;
    assert(CalibrationStore_Save(&d)==CAL_STORE_OK);bad=d;bad.gyro_bias[0]=.02f;
    fail_after=failure;assert(CalibrationStore_Save(&bad)==CAL_STORE_IO_ERROR);
    assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&!memcmp(&d,&out,sizeof(d)));
    fail_after=-1;assert(CalibrationStore_Save(&bad)==CAL_STORE_OK);
    assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&!memcmp(&bad,&out,sizeof(bad)));
  }
  memset(calibration_test_flash,255,sizeof(calibration_test_flash));fail_after=-1;
  assert(CalibrationStore_Save(&d)==CAL_STORE_OK);
  MatrixRecord m={0};m.magic=0x43414C31;m.version=2;m.sequence=2;m.size=sizeof(m.data);m.commit=0xC011CA1B;
  m.data.layout_tag=0xF4053102;m.data.flags=m.data.measured_flags=7;m.data.who=0x70;m.data.type=2;
  memcpy(m.data.ao,d.accel_offset,12);memcpy(m.data.as,d.accel_scale,12);memcpy(m.data.mo,d.mag_offset,12);memcpy(m.data.gb,d.gyro_bias,12);
  for(unsigned i=0;i<3;i++)m.data.matrix[i*4]=d.mag_scale[i];
  m.data.gb[0]=.08f;commit_matrix(&m);d.gyro_bias[0]=.08f;
  assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&!memcmp(&d,&out,sizeof(d)));
  saved=programs;assert(CalibrationStore_Save(&out)==CAL_STORE_OK&&programs==saved);
  m.data.matrix[1]=.1f;commit_matrix(&m);
  assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&out.gyro_bias[0]!=.08f); /* Never silently discard cross-axis terms. */
  m.data.matrix[1]=0;commit_matrix(&m);calibration_test_flash[128+28]^=1;
  assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&out.gyro_bias[0]!=.08f);
  commit_matrix(&m);d.gyro_bias[1]=.03f;assert(CalibrationStore_Save(&d)==CAL_STORE_OK);
  uint32_t seq;memcpy(&seq,calibration_test_flash+256+8,4);assert(seq==3);
  assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&!memcmp(&d,&out,sizeof(d)));
  memset(calibration_test_flash+384,0,sizeof(calibration_test_flash)-384);
  bad=d;bad.gyro_bias[1]=.04f;assert(CalibrationStore_Save(&bad)==CAL_STORE_FULL);
  assert(CalibrationStore_Load(&out)==CAL_STORE_OK&&!memcmp(&d,&out,sizeof(d)));
  if(argc>1){
    FILE *f=fopen(argv[1],"rb");assert(f);assert(fread(calibration_test_flash,1,sizeof(calibration_test_flash),f)==sizeof(calibration_test_flash));fclose(f);
    assert(CalibrationStore_Load(&out)==CAL_STORE_OK);assert(out.flags==7&&out.measured_flags==6);
    assert(fabsf(out.gyro_bias[0]-.018768450245261192f)<1e-8f);
    assert(fabsf(out.gyro_bias[1]-.050822388380765915f)<1e-8f);
    assert(fabsf(out.mag_scale[0]-.7314968705177307f)<1e-7f);
    saved=programs;assert(CalibrationStore_Save(&out)==CAL_STORE_OK&&programs==saved);
    puts("PASS archived mixed v1/v2 journal: latest gyro bias preserved; no Flash write on load/dedup");
  }
  puts("PASS v1/v2 diagonal calibration compatibility, CRC, full journal, all interrupted writes, dedup, sequence and unsupported matrix rejection");
}
