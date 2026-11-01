#ifndef ADCS_MATH_H
#define ADCS_MATH_H
#include <math.h>

/* Hamilton [w,x,y,z], active body-to-world, intrinsic ZYX Euler angles. */
static inline int ADCS_AttitudeError(const float measured[4],const float target_deg[3],float body_deg[3])
{
  const float half_rad=0.008726646259971648f;
  float cr=cosf(target_deg[0]*half_rad),sr=sinf(target_deg[0]*half_rad);
  float cp=cosf(target_deg[1]*half_rad),sp=sinf(target_deg[1]*half_rad);
  float cy=cosf(target_deg[2]*half_rad),sy=sinf(target_deg[2]*half_rad);
  float t[4]={cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
              cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
  float n=0;for(unsigned i=0;i<4;i++)n+=measured[i]*measured[i];
  if(!isfinite(n)||n<0.5f||n>1.5f)return 0;
  float q[4];for(unsigned i=0;i<4;i++)q[i]=measured[i]/sqrtf(n);
  /* conj(q_measured) * q_target: desired rotation resolved in current body. */
  float e[4]={q[0]*t[0]+q[1]*t[1]+q[2]*t[2]+q[3]*t[3],
    q[0]*t[1]-q[1]*t[0]-q[2]*t[3]+q[3]*t[2],
    q[0]*t[2]+q[1]*t[3]-q[2]*t[0]-q[3]*t[1],
    q[0]*t[3]-q[1]*t[2]+q[2]*t[1]-q[3]*t[0]};
  if(e[0]<0)for(unsigned i=0;i<4;i++)e[i]=-e[i]; /* shortest rotation */
  float v=sqrtf(e[1]*e[1]+e[2]*e[2]+e[3]*e[3]);
  float scale=v>1e-7f?114.591559f*atan2f(v,e[0])/v:114.591559f;
  for(unsigned i=0;i<3;i++){body_deg[i]=e[i+1]*scale;if(!isfinite(body_deg[i]))return 0;}
  return 1;
}

/* Right multiplicative ESKF error reset: G_theta=I-0.5*[delta_theta]x. */
static inline void ADCS_CovarianceReset(float p[6][6],const float theta[3])
{
  float g[6][6]={{0}},gp[6][6]={{0}},r[6][6]={{0}};
  for(unsigned i=0;i<6;i++)g[i][i]=1;
  g[0][1]=.5f*theta[2];g[0][2]=-.5f*theta[1];
  g[1][0]=-.5f*theta[2];g[1][2]=.5f*theta[0];
  g[2][0]=.5f*theta[1];g[2][1]=-.5f*theta[0];
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)for(unsigned k=0;k<6;k++)gp[i][j]+=g[i][k]*p[k][j];
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)for(unsigned k=0;k<6;k++)r[i][j]+=gp[i][k]*g[j][k];
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)p[i][j]=.5f*(r[i][j]+r[j][i]);
}
#endif
