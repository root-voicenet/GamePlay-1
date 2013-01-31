#include "LineDrawer.h"

using namespace gameplay;

LineDrawer::LineDrawer()
    : _meshBatch(NULL), _lineCount(0)
{
    // Vertex shader for drawing colored lines.
    const char* vs_str = 
    {
        "uniform mat4 u_viewProjectionMatrix;\n"
        "attribute vec4 a_position;\n"
        "attribute vec4 a_color;\n"
        "varying vec4 v_color;\n"
        "void main(void) {\n"
        "    v_color = a_color;\n"
        "    gl_Position = u_viewProjectionMatrix * a_position;\n"
        "}"
    };

    // Fragment shader for drawing colored lines.
    const char* fs_str = 
    {
    #ifdef OPENGL_ES
        "precision highp float;\n"
    #endif
        "varying vec4 v_color;\n"
        "void main(void) {\n"
        "   gl_FragColor = v_color;\n"
        "}"
    };

    Effect* effect = Effect::createFromSource(vs_str, fs_str);
    Material* material = Material::create(effect);
    GP_ASSERT(material && material->getStateBlock());
    material->getStateBlock()->setDepthTest(true);
    material->getStateBlock()->setDepthFunction(RenderState::DEPTH_LEQUAL);

    VertexFormat::Element elements[] =
    {
        VertexFormat::Element(VertexFormat::POSITION, 3),
        VertexFormat::Element(VertexFormat::COLOR, 4),
    };
    _meshBatch = MeshBatch::create(VertexFormat(elements, 2), Mesh::LINES, material, false, 4096, 4096);
    SAFE_RELEASE(material);
    SAFE_RELEASE(effect);
}

LineDrawer::~LineDrawer()
{
    SAFE_DELETE(_meshBatch);
}

void LineDrawer::begin(const Matrix& viewProjection)
{
    GP_ASSERT(_meshBatch);
    _meshBatch->start();
    _meshBatch->getMaterial()->getParameter("u_viewProjectionMatrix")->setValue(viewProjection);
}

void LineDrawer::end()
{
    GP_ASSERT(_meshBatch && _meshBatch->getMaterial());
    _meshBatch->finish();
    _meshBatch->draw();
    _lineCount = 0;
}

void LineDrawer::drawLine(const Vector3& from, const Vector3& to, const Vector3& fromColor, const Vector3& toColor)
{
    GP_ASSERT(_meshBatch);

    static LineDrawer::LineVertex vertices[2];

	vertices[0].x = from.x;
    vertices[0].y = from.y;
    vertices[0].z = from.z;
    vertices[0].r = fromColor.x;
    vertices[0].g = fromColor.y;
    vertices[0].b = fromColor.z;
    vertices[0].a = 1.0f;

    vertices[1].x = to.x;
    vertices[1].y = to.y;
    vertices[1].z = to.z;
    vertices[1].r = toColor.x;
    vertices[1].g = toColor.y;
    vertices[1].b = toColor.z;
    vertices[1].a = 1.0f;

    _meshBatch->add(vertices, 2);

    ++_lineCount;
    if (_lineCount >= 4096)
    {
        // Flush the batch when it gets full (don't want to to grow infinitely)
        end();
        _meshBatch->start();
    }
}

void LineDrawer::drawLine(const Vector3& from, const Vector3& to, const Vector3& color)
{
    drawLine(from, to, color, color);
}

void LineDrawer::drawContactPoint(const Vector3& pointOnB, const Vector3& normalOnB, double distance, int lifeTime, const Vector3& color)
{
    drawLine(pointOnB, pointOnB + normalOnB, color);
}
