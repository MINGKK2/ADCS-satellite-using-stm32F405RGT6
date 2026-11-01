#ifndef HEADING_UPDATE_H
#define HEADING_UPDATE_H
#include <math.h>
#include <stdint.h>
#include "adcs_math.h"
/* Configured observation-noise assumptions, not measured sensor accuracy.
 * Only sustained clean, quiet samples receive the stronger heading weight. */
static float Heading_NoiseVariance(float field_ratio,float motor_activity,uint8_t stable_clean) {
  (void)stable_clean;
  /* Clean-looking samples alone cannot establish absolute accuracy. */
  float sigma=0.14f;
  float deviation=fabsf(field_ratio-1.0f)/0.28f;
  return sigma*sigma*(1.0f+4.0f*deviation*deviation)
      *(1.0f+24.0f*motor_activity*motor_activity);
}
/* Scalar heading observation H=[gravity_body,0,0,0]. Full six-state gain
 * makes gyro heading bias observable. Joseph covariance uses the actual
 * gain after restricting the attitude correction to the heading axis. */
static int Heading_Update(float p[6][6],const float gravity[3],float residual,
                          float noise,uint8_t learn_bias,float theta[3],float bias[3]) {
  float ph[6]={0},h[6]={gravity[0],gravity[1],gravity[2],0,0,0},k[6];
  float s=noise;
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)ph[i]+=p[i][j]*h[j];
  for(unsigned i=0;i<6;i++)s+=h[i]*ph[i];
  if(!isfinite(s) || s<1e-9f)return 0;
  for(unsigned i=0;i<6;i++)k[i]=ph[i]/s;
  float yaw_gain=0;
  for(unsigned i=0;i<3;i++)yaw_gain+=k[i]*gravity[i];
  for(unsigned i=0;i<3;i++)k[i]=yaw_gain*gravity[i];
  float angle_step=fabsf(yaw_gain*residual);
  if(angle_step>0.035f)for(unsigned i=0;i<6;i++)k[i]*=0.035f/angle_step;
  float largest_bias_step=0;
  for(unsigned i=3;i<6;i++)largest_bias_step=fmaxf(largest_bias_step,fabsf(k[i]*residual));
  for(unsigned i=3;i<6;i++) {
    if(!learn_bias)k[i]=0;
    else if(largest_bias_step>0.00005f)k[i]*=0.00005f/largest_bias_step;
  }
  for(unsigned i=0;i<3;i++){theta[i]=k[i]*residual;bias[i]=k[i+3]*residual;}
  float a[6][6],ap[6][6]={0},updated[6][6]={0};
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)a[i][j]=(i==j?1.0f:0.0f)-k[i]*h[j];
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)for(unsigned m=0;m<6;m++)ap[i][j]+=a[i][m]*p[m][j];
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++) {
    for(unsigned m=0;m<6;m++)updated[i][j]+=ap[i][m]*a[j][m];
    updated[i][j]+=k[i]*noise*k[j];
  }
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)p[i][j]=0.5f*(updated[i][j]+updated[j][i]);
  for(unsigned i=0;i<6;i++)p[i][i]=fmaxf(p[i][i],1e-9f);
  ADCS_CovarianceReset(p,theta);
  return 1;
}
#endif
