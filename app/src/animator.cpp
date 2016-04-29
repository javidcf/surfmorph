
#include "animator.h"

#include <Eigen/Dense>
#include <QFile>
#include <QGLViewer/manipulatedCameraFrame.h>
#include <QGLViewer/vec.h>
#include <QKeyEvent>
#include <QtGlobal>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include <iostream>

const Animator::Vec3 Animator::MeshColor{Scalar(1), Scalar(1), Scalar(1)};

Animator::Animator(const QStringList &poseFiles, Animator::Scalar duration,
                   Animator::Scalar fps)
: QGLViewer()
, m_fps{fps}
, m_duration{duration}
, m_speed{0}
, m_backwards{false}
, m_framePoses()
, m_triangles()
, m_numVertices{0}
, m_currentFrame{0}
, m_vao(this)
, m_vertexVbo(QOpenGLBuffer::VertexBuffer)
, m_normalVbo(QOpenGLBuffer::VertexBuffer)
, m_colorVbo(QOpenGLBuffer::VertexBuffer)
, m_elemVbo(QOpenGLBuffer::IndexBuffer)
, m_program(this)
, m_posAttrLoc{-1}
, m_normalAttrLoc{-1}
, m_colorAttrLoc{-1}
, m_projMatAttrLoc{-1}
, m_modelViewMatAttrLoc{-1}
{
    Eigen::initParallel();
    if (poseFiles.size() < 2)
    {
        throw std::runtime_error("At least two poses are necessary");
    }
    if (fps <= Scalar(0))
    {
        throw std::runtime_error("The frame rate must be a positive value");
    }
    if (duration <= Scalar(0))
    {
        throw std::runtime_error("The duration must be a positive value");
    }

    // TODO do this in a different method while showing progress?
    qInfo() << "Reading pose files...";
    std::vector<std::shared_ptr<Mesh>> poses;
    poses.reserve(poseFiles.size());
    for (const auto &file : poseFiles) {
        poses.push_back(Animator::readMesh(file));
    }
    generateAnimation(poses);
}

Animator::~Animator()
{
    destroyVertexBuffers();
}

void Animator::init()
{
    setWindowTitle("Animator");

    initializeOpenGLFunctions();

    // Disable spinning
    camera()->frame()->setSpinningSensitivity(100.0);

    if (!hasOpenGLFeature(QOpenGLFunctions::Shaders) ||
        !hasOpenGLFeature(QOpenGLFunctions::Buffers))
    {
        throw std::runtime_error("Unsupported version of OpenGL");
    }

    setBackgroundColor(Qt::black);

    // Shaders
    loadShaders();
    createVertexBuffers();

    restoreStateFromFile();

    // Camera
    setupCamera();

    // Control
    setKeyDescription(Qt::Key_B, "Toggle backwards animation");
    setKeyDescription(Qt::Key_Plus, "Accelerate animation");
    setKeyDescription(Qt::Key_Minus, "Decelerate animation");

    // Set up animation
    allocateVertexBuffers();
    updateAnimationPeriod();
    startAnimation();
}

void Animator::loadShaders()
{

    m_program.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                      ":/shaders/vertex.glsl");
    m_program.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                      ":/shaders/fragment.glsl");
    m_program.link();

    // Attribute locations
    m_posAttrLoc = m_program.attributeLocation("position");
    m_normalAttrLoc = m_program.attributeLocation("normal");
    m_colorAttrLoc = m_program.attributeLocation("color");
    m_projMatAttrLoc = m_program.uniformLocation("projection");
    m_modelViewMatAttrLoc = m_program.uniformLocation("modelView");
}

void Animator::createVertexBuffers()
{
    m_vertexVbo.bind();
    m_vertexVbo.create();
    m_vertexVbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    m_normalVbo.create();
    m_normalVbo.bind();
    m_normalVbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    m_colorVbo.create();
    m_colorVbo.bind();
    m_colorVbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    m_elemVbo.create();
    m_elemVbo.bind();
    m_elemVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
}

void Animator::allocateVertexBuffers()
{
    auto vertexBufferSize = m_numVertices * 3 * sizeof(Scalar);

    // Initialize color
    std::vector<Vec3> colorData;
    colorData.reserve(m_numVertices);
    for (Idx iVertex = 0; iVertex < m_numVertices; iVertex++) {
        colorData.push_back(MeshColor);
    }
    m_colorVbo.bind();
    m_colorVbo.allocate(colorData.data(), vertexBufferSize);
    m_colorVbo.release();

    // Copy triangles data
    auto elementBufferSize = m_triangles.size() * sizeof(Idx);
    m_elemVbo.bind();
    m_elemVbo.allocate(m_triangles.data(), elementBufferSize);
    m_elemVbo.release();

    // Set first frame
    setFrame(0);

    bindVao();
}

void Animator::destroyVertexBuffers()
{
    m_vertexVbo.destroy();
    m_normalVbo.destroy();
    m_colorVbo.destroy();
    m_elemVbo.destroy();
}

void Animator::bindVao()
{
    // Bind VAO layout
    m_vao.bind();
    m_program.bind();

    m_program.enableAttributeArray(m_posAttrLoc);
    m_vertexVbo.bind();
    m_program.setAttributeBuffer(m_posAttrLoc, ScalarTypeEnum, 0, 3);
    m_vertexVbo.release();

    m_program.enableAttributeArray(m_normalAttrLoc);
    m_normalVbo.bind();
    m_program.setAttributeBuffer(m_normalAttrLoc, ScalarTypeEnum, 0, 3);
    m_normalVbo.release();

    m_program.enableAttributeArray(m_colorAttrLoc);
    m_colorVbo.bind();
    m_program.setAttributeBuffer(m_colorAttrLoc, ScalarTypeEnum, 0, 3);
    m_colorVbo.release();

    m_elemVbo.bind();

    m_program.release();
    m_vao.release();
}

void Animator::generateAnimation(
    const std::vector<std::shared_ptr<Animator::Mesh>> &poses)
{
    const auto &basePose = *poses[0];
    // Copy triangles structure
    m_triangles.reserve(basePose.n_faces() * 3);
    for (auto faceIt = basePose.faces_begin(); faceIt != basePose.faces_end();
         ++faceIt)
    {
        for (auto fvIt = basePose.cfv_iter(*faceIt); fvIt.is_valid(); ++fvIt) {
            m_triangles.push_back(fvIt->idx());
        }
    }
    m_numVertices = basePose.n_vertices();
    SurfaceMorph surfaceMorph = createSurfaceMorph(poses);
    generateAnimationFrames(basePose, surfaceMorph);

    qInfo() << "Animation generated";
}

Animator::SurfaceMorph Animator::createSurfaceMorph(
    const std::vector<std::shared_ptr<Animator::Mesh>> &poses)
{
    qInfo() << "Creating interpolator...";

    using namespace Eigen;

    if (poses.size() < 2)
    {
        throw std::runtime_error("At least two poses are necessary");
    }

    auto firstPose = poses[0];

    // Check every pose has the same number of vertices, edges and faces
    for (const auto &pose : poses) {
        if (pose->n_vertices() != firstPose->n_vertices() ||
            pose->n_halfedges() != firstPose->n_halfedges() ||
            pose->n_faces() != firstPose->n_faces())
        {
            throw std::runtime_error(
                "Every pose mesh must have the same structure.");
        }
    }

    // Create triangles matrix
    Matrix<Idx, 3, Dynamic> triangles(3, firstPose->n_faces());
    // Could parallelize, worth it?
    for (auto faceIt = firstPose->faces_begin();
         faceIt != firstPose->faces_end(); ++faceIt)
    {
        Idx iVertex = 0;
        for (auto fvIt = firstPose->fv_iter(*faceIt); fvIt.is_valid(); ++fvIt) {
            triangles(iVertex, faceIt->idx()) = static_cast<Idx>(fvIt->idx());
            iVertex++;
        }
    }

    // Create the surface morpher
    SurfaceMorph surfaceMorph(triangles);

    // Add poses
    for (const auto &pose : poses) {
        Matrix<Scalar, 3, Dynamic> poseCoords(3, pose->n_vertices());
        for (auto vertexIt = pose->vertices_begin();
             vertexIt != pose->vertices_end(); ++vertexIt)
        {
            const auto &p = pose->point(*vertexIt);
            poseCoords.col(vertexIt->idx()) << p[0], p[1], p[2];
        }
        surfaceMorph.addPose(poseCoords);
    }

    return surfaceMorph;
}

void Animator::generateAnimationFrames(
    const Animator::Mesh &basePose, const Animator::SurfaceMorph &surfaceMorph)
{
    using namespace Eigen;

    int numFrames = static_cast<int>(m_duration * m_fps);
    if (numFrames < 0)
    {
        throw std::runtime_error(
            "The duration and frame rate must be positive values");
    }

    // Interpolate poses
    m_framePoses.resize(numFrames);
    auto t = Matrix<Scalar, Dynamic, 1>::LinSpaced(
        numFrames, Scalar(0), Scalar(surfaceMorph.numPoses() - 1));
    int processedFrames = 0;
    int tenPercents = -1;
#pragma omp parallel
    {
        Mesh mutablePose{basePose};
#pragma omp for
        for (int iFrame = 0; iFrame < numFrames; iFrame++) {
            m_framePoses[iFrame] =
                generateFramePose(mutablePose, surfaceMorph, t[iFrame]);
#pragma omp critical
            {
                processedFrames++;
                int currentTen = (processedFrames * 10) / numFrames;
                if (currentTen > tenPercents)
                {
                    tenPercents = currentTen;
                    qInfo("Interpolating frames (%d%%)...", tenPercents * 10);
                }
            }
        }
    }
}

Animator::FramePose Animator::generateFramePose(
    Mesh &mutablePose, const SurfaceMorph &surfaceMorph, Scalar t)
{
    using namespace Eigen;

    // Compute interpolation
    Matrix<Scalar, 3, Dynamic> framePose = surfaceMorph.interpolatePoseAt(t);

    // Copy to mesh to compute normals
    for (auto vIt = mutablePose.vertices_begin();
         vIt != mutablePose.vertices_end(); ++vIt)
    {
        auto vIdx = vIt->idx();
        mutablePose.set_point(*vIt, Mesh::Point(framePose(0, vIdx),
                                                framePose(1, vIdx),
                                                framePose(2, vIdx)));
    }
    mutablePose.update_normals();

    // Save frame
    auto numVertices = mutablePose.n_vertices();
    std::vector<Scalar> coords;
    coords.reserve(numVertices * 3);
    std::vector<Scalar> normals;
    normals.reserve(numVertices * 3);
    for (auto vIt = mutablePose.vertices_begin();
         vIt != mutablePose.vertices_end(); ++vIt)
    {
        auto p = mutablePose.point(*vIt);
        auto n = mutablePose.normal(*vIt);
        coords.push_back(p[0]);
        coords.push_back(p[1]);
        coords.push_back(p[2]);
        normals.push_back(n[0]);
        normals.push_back(n[1]);
        normals.push_back(n[2]);
    }

    return {std::move(coords), std::move(normals)};
}

std::shared_ptr<Animator::Mesh> Animator::readMesh(const QString &meshFile)
{
    std::shared_ptr<Mesh> newMesh = std::make_shared<Mesh>();
    newMesh->request_vertex_normals();
    newMesh->request_face_normals();
    OpenMesh::IO::Options ropt;
    if (!OpenMesh::IO::read_mesh(*newMesh, meshFile.toStdString(), ropt))
    {
        throw std::runtime_error("Could not load mesh file.");
    }
    // If the file did not provide vertex normals, then calculate them
    if (!ropt.check(OpenMesh::IO::Options::VertexNormal))
    {
        newMesh->update_normals();
    }
    return newMesh;
}

void Animator::setFrame(unsigned int iFrame)
{
    auto vertexBufferSize = m_numVertices * 3 * sizeof(Scalar);
    const auto &framePose = m_framePoses[iFrame];

    m_vertexVbo.bind();
    m_vertexVbo.allocate(nullptr, vertexBufferSize);
    auto vertexVboMapped = m_vertexVbo.map(QOpenGLBuffer::WriteOnly);
    std::memcpy(vertexVboMapped, framePose.coords.data(), vertexBufferSize);
    m_vertexVbo.unmap();
    m_vertexVbo.release();

    m_normalVbo.bind();
    m_normalVbo.allocate(nullptr, vertexBufferSize);
    auto normalVboMapped = m_vertexVbo.map(QOpenGLBuffer::WriteOnly);
    std::memcpy(normalVboMapped, framePose.normals.data(), vertexBufferSize);
    m_vertexVbo.unmap();
    m_normalVbo.release();
}

void Animator::setupCamera()
{
    // Find poses bounds
    qglviewer::Vec pMin(std::numeric_limits<qreal>::max(),
                        std::numeric_limits<qreal>::max(),
                        std::numeric_limits<qreal>::max());
    qglviewer::Vec pMax(std::numeric_limits<qreal>::min(),
                        std::numeric_limits<qreal>::min(),
                        std::numeric_limits<qreal>::min());
    for (const auto &pose : m_framePoses) {
        for (Idx iCoord = 0; iCoord < pose.coords.size(); iCoord += 3) {
            pMin[0] = std::min(qreal(pose.coords[iCoord + 0]), pMin[0]);
            pMin[1] = std::min(qreal(pose.coords[iCoord + 1]), pMin[1]);
            pMin[2] = std::min(qreal(pose.coords[iCoord + 2]), pMin[2]);
            pMax[0] = std::max(qreal(pose.coords[iCoord + 0]), pMax[0]);
            pMax[1] = std::max(qreal(pose.coords[iCoord + 1]), pMax[1]);
            pMax[2] = std::max(qreal(pose.coords[iCoord + 2]), pMax[2]);
        }
    }

    // Set camera direction
    camera()->setUpVector(qglviewer::Vec(0, 1, 0));
    camera()->setViewDirection(qglviewer::Vec(0, 0, -1));

    // Set scene bounds
    camera()->setSceneBoundingBox(pMin, pMax);
    camera()->fitBoundingBox(pMin, pMax);
}

void Animator::updateAnimationPeriod()
{
    bool started{animationIsStarted()};

    auto speedFactor = static_cast<Scalar>(std::pow(1.2, m_speed));
    auto newPeriod =
        std::max(Scalar(0),
                 std::min(static_cast<Scalar>(std::numeric_limits<int>::max()),
                          std::round(Scalar(1000) / (m_fps * speedFactor))));
    if (started)
    {
        stopAnimation();
    }
    setAnimationPeriod(static_cast<int>(newPeriod));
    if (started)
    {
        startAnimation();
    }
}

void Animator::animate()
{
    if (m_framePoses.size() < 1)
    {
        return;
    }

    unsigned int animationFrame;
    if (m_backwards && m_framePoses.size() > 1)
    {
        m_currentFrame = m_currentFrame % (2 * m_framePoses.size() - 2);
        if (m_currentFrame < m_framePoses.size())
        {
            animationFrame = m_currentFrame;
        }
        else
        {
            animationFrame = 2 * m_framePoses.size() - m_currentFrame - 2;
        }
    }
    else
    {
        m_currentFrame = m_currentFrame % m_framePoses.size();
        animationFrame = m_currentFrame;
    }
    setFrame(animationFrame);
    bindVao();
    m_currentFrame++;
}

void Animator::preDraw()
{
    QGLViewer::preDraw();

    // Get cameras
    Scalar pm[4][4];
    camera()->getProjectionMatrix(reinterpret_cast<Scalar *>(pm));
    Scalar mvm[4][4];
    camera()->getModelViewMatrix(reinterpret_cast<Scalar *>(mvm));

    // Set cameras
    m_program.bind();
    m_program.setUniformValue(m_projMatAttrLoc, pm);
    m_program.setUniformValue(m_modelViewMatAttrLoc, mvm);
}

void Animator::preDrawStereo(bool leftBuffer)
{
    QGLViewer::preDrawStereo(leftBuffer);

    // Get cameras
    Scalar pm[4][4];
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    camera()->loadProjectionMatrixStereo(leftBuffer);
    glGetFloatv(GL_PROJECTION_MATRIX, reinterpret_cast<Scalar *>(pm));
    glPopMatrix();

    Scalar mvm[4][4];
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    camera()->loadModelViewMatrixStereo(leftBuffer);
    glGetFloatv(GL_MODELVIEW_MATRIX, reinterpret_cast<Scalar *>(mvm));
    glPopMatrix();

    // Set cameras
    m_program.bind();
    m_program.setUniformValue(m_projMatAttrLoc, pm);
    m_program.setUniformValue(m_modelViewMatAttrLoc, mvm);
}

void Animator::draw()
{
    QGLViewer::draw();

    if (m_framePoses.size() < 1)
    {
        return;
    }

    m_vao.bind();
    glDrawElements(GL_TRIANGLES, m_triangles.size(), IdxTypeEnum, nullptr);
    m_vao.release();
}

void Animator::postDraw()
{
    m_program.release();

    QGLViewer::postDraw();
}

void Animator::keyPressEvent(QKeyEvent *e)
{
    auto key = e->key();
    if (key == Qt::Key_B)
    {
        // Backwards animation
        m_backwards = !m_backwards;
        if (!m_backwards && m_currentFrame >= m_framePoses.size())
        {
            m_currentFrame = 0;
        }
    }
    if (key == Qt::Key_Plus)
    {
        m_speed++;
        updateAnimationPeriod();
    }
    else if (key == Qt::Key_Minus)
    {
        m_speed--;
        updateAnimationPeriod();
    }
    else
    {
        QGLViewer::keyPressEvent(e);
    }
}

QString Animator::helpString() const
{
    QFile helpFile(":/doc/help.html");
    if (!helpFile.open(QFile::ReadOnly | QFile::Text))
    {
        throw std::logic_error("Error opening internal resource");
    }
    QTextStream helpStream(&helpFile);
    QString helpText = helpStream.readAll();
    helpFile.close();
    return helpText;
}
