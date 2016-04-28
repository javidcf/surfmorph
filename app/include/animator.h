
#ifndef ANIMATOR_H
#define ANIMATOR_H

#include <OpenMesh/Core/IO/MeshIO.hh> // Before any mesh type
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <QGLShaderProgram>
#include <QGLViewer/qglviewer.h>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QString>
#include <QStringList>
#include <surfmorph/surfmorph.h>

#include <array>
#include <memory>
#include <vector>

//!
//! \brief Mesh animator.
//!
//! Animates a mesh by interpolating several poses. The poses must be mesh files
//! stored in disk, and vertex indices are assumed to be registered.
//!
class Animator : public QGLViewer, QOpenGLFunctions
{
public:
    //! Scalar type
    typedef GLfloat Scalar;
    //! Index type
    typedef GLuint Idx;

    //!
    //! \brief Constructor.
    //!
    //! \param poseFiles Mesh files with the animation poses
    //! \param duration Animation duration in seconds
    //! \param fps Animation frame rate in frames per second
    //!
    Animator(const QStringList &poseFiles, Scalar duration, Scalar fps);

    //!
    //! \brief Destructor.
    //!
    virtual ~Animator();

protected:
    virtual void animate();
    virtual void preDraw();
    virtual void preDrawStereo(bool leftBuffer = true);
    virtual void draw();
    virtual void postDraw();
    virtual void init();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual QString helpString() const;

private:
    //! Surface morpher
    typedef surfmorph::SurfaceMorph<Scalar> SurfaceMorph;

    //! \brief Information of a pose at a frame.
    struct FramePose
    {
        std::vector<Scalar> coords;
        std::vector<Scalar> normals;
    };

    //!
    //! Custom mesh traits
    //!
    struct MeshTraits : public OpenMesh::DefaultTraits
    {
        // VertexAttributes(OpenMesh::DefaultAttributer::Normal |
        //                  OpenMesh::DefaultAttributer::Color);
        // HalfedgeAttributes(OpenMesh::DefaultAttributer::PrevHalfedge);
        // FaceAttributes(OpenMesh::DefaultAttributer::Normal);
    };

    typedef OpenMesh::TriMesh_ArrayKernelT<MeshTraits> Mesh;

    //! A vector of 3 GLfloat values
    typedef std::array<Scalar, 3> Vec3;

    //! OpenGL type enum for the scalar (coordinate) type
    static const GLenum ScalarTypeEnum{GL_FLOAT};
    //! OpenGL type enum for the index type
    static const GLenum IdxTypeEnum{GL_UNSIGNED_INT};

    //! Mesh color
    static const Vec3 MeshColor;

    //! Animation frame rate
    const Scalar m_fps;
    //! Animation duration
    const Scalar m_duration;
    //! Animation play speed
    int m_speed;
    //! Whether to play the animation backwards on each loop
    bool m_backwards;
    //! Animation frame poses
    std::vector<FramePose> m_framePoses;
    //! Mesh triangles (as OpenGL data buffer)
    std::vector<Idx> m_triangles;
    //! Number of vertices
    Idx m_numVertices;
    //! Current animation frame
    unsigned int m_currentFrame;

    //! OpenGL VAO
    QOpenGLVertexArrayObject m_vao;
    //! Vertex coordinates VBO
    QOpenGLBuffer m_vertexVbo;
    //! Vertex normal VBO
    QOpenGLBuffer m_normalVbo;
    //! Vertex color VBO
    QOpenGLBuffer m_colorVbo;
    //! Elements (triangles) VBO
    QOpenGLBuffer m_elemVbo;
    //! OpenGL program
    QOpenGLShaderProgram m_program;
    //! Vertex coordinates attribute location
    int m_posAttrLoc;
    //! Vertex normal attribute location
    int m_normalAttrLoc;
    //! Vertex color attribute location
    int m_colorAttrLoc;
    //! Projection matrix attribute location
    int m_projMatAttrLoc;
    //! Model-view matrix attribute location
    int m_modelViewMatAttrLoc;

    //!
    //! \brief Generate the animation
    //!
    //! \param poses Meshes representing the poses to be interpolated
    //!
    void generateAnimation(const std::vector<std::shared_ptr<Mesh>> &poses);

    //!
    //! \brief Read a mesh file.
    //!
    //! \param meshFile The path of the mesh file
    //! \return The read mesh file
    //!
    static std::shared_ptr<Mesh> readMesh(const QString &meshFile);

    //!
    //! \brief Create surface morph.
    //!
    //! \param poses Meshes representing the poses to be interpolated
    //!
    static SurfaceMorph
    createSurfaceMorph(const std::vector<std::shared_ptr<Mesh>> &poses);

    //!
    //! \brief Create the poses of each animation frame.
    //!
    //! \param basePose A mesh holding the triangle structure
    //! \param SurfaceMorph The surface morpher used to generate the animation
    //!
    void generateAnimationFrames(const Mesh &basePose,
                                 const SurfaceMorph &SurfaceMorph);

    //!
    //! \brief Create a frame pose at a given time point.
    //!
    //! \param mutablePose A mutable mesh holding the triangle structure
    //! \param surfaceMorph The surface morpher used to generate the animation
    //! \param t Time point of the frame
    //!
    //! \return A new frame pose for the given time point
    //!
    FramePose generateFramePose(Mesh &mutablePose,
                                const SurfaceMorph &surfaceMorph, Scalar t);

    //!
    //! \brief Load OpenGL shaders.
    //!
    void loadShaders();

    //!
    //! \brief Create OpenGL vertex buffers.
    //!
    void createVertexBuffers();

    //!
    //! \brief Allocate OpenGL vertex buffers.
    //!
    void allocateVertexBuffers();

    //!
    //! \brief Destroy OpenGL vertex buffers.
    //!
    void destroyVertexBuffers();

    //!
    //! \brief Bind parameters of OpenGL VBO
    //!
    void bindVao();

    //!
    //! \brief Set the current frame.
    //!
    //! \param iFrame Index of the frame to be set
    //!
    void setFrame(unsigned int iFrame);

    //!
    //! \brief Configure the camera to fit the animation.
    //!
    void setupCamera();

    //!
    //! \brief Set the animation perio according to the current speed.
    //!
    void updateAnimationPeriod();
};

#endif
