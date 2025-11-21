#ifndef ESO_H
#define ESO_H

#include <QtMath>
class ESO
{
public:
    ESO();
    void eso_calculate(float psi,float m_r,float tau_n, float r);
    float z1,z2,z3,h,w0,psi_last;
private:

};

#endif // ESO_H
