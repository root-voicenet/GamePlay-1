#pragma once
#include "gameplay.h"

namespace gameplay 
{

class LineDrawer
{
public:

    struct LineVertex
    {
        float x;
        float y;
        float z;

        float r;
        float g;
        float b;
        float a;
    };

    LineDrawer(); 
    ~LineDrawer();
        
    void begin(const Matrix& viewProjection);
    void end();


    void drawLine(const Vector3& from, const Vector3& to, const Vector3& fromColor, const Vector3& toColor);        
    void drawLine(const Vector3& from, const Vector3& to, const Vector3& color);        
    void drawContactPoint(const Vector3& pointOnB, const Vector3& normalOnB, double distance, int lifeTime, const Vector3& color);        

        
private:
    MeshBatch* _meshBatch;
    int _lineCount;
};

}