#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1
    if (points.size() == 1) {
      return points;
    }
    
    vector<Vector2D> intermediatePoints;

    for (int i = 0; i < points.size() - 1; i++) {
      Vector2D current = (1.0f - t) * points[i] + t * points[i + 1];
      intermediatePoints.push_back(current);
    }

    return intermediatePoints;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2
    if (points.size() == 1) {
      return points;
    }

    vector<Vector3D> intermediatePoints;

    for (int i = 0; i < points.size() - 1; i++) {
      Vector3D current = (1.0f - t) * points[i] + t * points[i + 1];
      intermediatePoints.push_back(current);
    }

    return intermediatePoints;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    // Use evaluateStep to get the final interpolated point
    if (points.size() == 1) {
      return points[0];
    }

    return evaluate1D(evaluateStep(points, t), t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> movingBezier;
    for (int i = 0; i < controlPoints.size(); i++) {
      movingBezier.push_back(evaluate1D(controlPoints[i], u));
    }

    return evaluate1D(movingBezier, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D accumulatedNormal(0, 0, 0);

    HalfedgeCIter he = this->halfedge();
    HalfedgeCIter heBegin = he;

    do {
        Vector3D faceNormal = he->face()->normal(); 
        
        accumulatedNormal += faceNormal;

        he = he->twin()->next();
    } while (he != heBegin);

    if (accumulatedNormal.norm() > 0) {
        accumulatedNormal.normalize(); 
    }

    return accumulatedNormal;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
   if (!e0->isBoundary()) {

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    FaceIter f1 = h0->face();
    FaceIter f2 = h3->face();

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

  

    h0->setNeighbors(h1, h3, v2, e0, f1);
    h1->setNeighbors(h2, h9, v3, e4, f1);
    h2->setNeighbors(h0, h6, v1, e1, f1);
    h3->setNeighbors(h4, h0, v3, e0, f2);
    h4->setNeighbors(h5, h7, v2, e2, f2);
    h5->setNeighbors(h3, h8, v0, e3, f2);
    h6->setNeighbors(h6->next(), h2, v2, e1, h6->face());
    h7->setNeighbors(h7->next(), h4, v0, e2, h7->face());
    h8->setNeighbors(h8->next(), h5, v3, e3, h8->face());
    h9->setNeighbors(h9->next(), h1, v1, e4, h9->face());

    v0->halfedge() = h5;
    v1->halfedge() = h2;
    v2->halfedge() = h4;
    v3->halfedge() = h1;
    f1->halfedge() = h0;
    f2->halfedge() = h3;
    e0->halfedge() = h0;
    e1->halfedge() = h2;
    e2->halfedge() = h4;
    e3->halfedge() = h5;
    e4->halfedge() = h1;


}

return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()) {
        return VertexIter();
    }

    // Definitions
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();

    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    FaceIter f1 = h0->face();
    FaceIter f2 = h3->face();

    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();

    VertexIter v = newVertex();
    v->position = (v0->position + v1->position) / 2.0;

    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

    FaceIter f3 = newFace();
    FaceIter f4 = newFace();
    h0->setNeighbors(h1, h3, v, e0, f1);
    h1->setNeighbors(h2, h6, v1, e1, f1);
    h2->setNeighbors(h0, h11, v2, e5, f1);
    h3->setNeighbors(h4, h0, v1, e0, f2);
    h4->setNeighbors(h5, h15, v, e7, f2);
    h5->setNeighbors(h3, h9, v3, e4, f2);
    h6->setNeighbors(h6->next(), h1, h6->vertex(), e1, h6->face());
    h7->setNeighbors(h7->next(), h12, h7->vertex(), e2, h7->face());
    h8->setNeighbors(h8->next(), h14, h8->vertex(), e3, h8->face());
    h9->setNeighbors(h9->next(), h5, h9->vertex(), e4, h9->face());
    h10->setNeighbors(h11, h13, v0, e6, f3);
    h11->setNeighbors(h12, h2, v, e5, f3);
    h12->setNeighbors(h10, h7, v2, e2, f3);
    h13->setNeighbors(h14, h10, v, e6, f4);
    h14->setNeighbors(h15, h8, v0, e3, f4);
    h15->setNeighbors(h13, h4, v3, e7, f4);

    v0->halfedge() = h10;
    v1->halfedge() = h1;
    v2->halfedge() = h12;
    v3->halfedge() = h5;
    v->halfedge() = h0;

    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h12;
    e3->halfedge() = h14;
    e4->halfedge() = h5;
    e5->halfedge() = h2;
    e6->halfedge() = h10;
    e7->halfedge() = h4;

    f1->halfedge() = h0;
    f2->halfedge() = h3;
    f3->halfedge() = h10;
    f4->halfedge() = h13;

    return v;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
      e->isNew = false; // Initialize isNew for edges

      HalfedgeIter h0 = e->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      // Calculate edge midpoint newPosition using adjacent vertices (v0, v1) and opposite vertices (v2, v3)
      VertexIter v0 = e->halfedge()->vertex();
      VertexIter v1 = e->halfedge()->twin()->vertex();
      VertexIter v2 = e->halfedge()->next()->next()->vertex();
      VertexIter v3 = e->halfedge()->twin()->next()->next()->vertex();
      e->newPosition = 3.0 / 8.0 * (v0->position + v1->position) + 1.0 / 8.0 * (v2->position + v3->position);
    }

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
        v->isNew = false; // Initialize isNew for vertices

        // Calculate new position for each vertex based on its neighbors
        Vector3D neighborPositionSum(0, 0, 0);
        float n = 0.0; // Number of neighbors (vertex degree)
        HalfedgeIter h = v->halfedge();
        do {
            neighborPositionSum += h->twin()->vertex()->position;
            n += 1;
            h = h->twin()->next();
        } while (h != v->halfedge());

        float u = (n == 3.0) ? 3.0 / 16.0 : 3.0 / (8.0 * n);
        v->newPosition = (1.0 - n * u) * v->position + u * neighborPositionSum;
    }
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    std::vector<EdgeIter> edgesToSplit;
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
        edgesToSplit.push_back(e);
    }


    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
        if (e->isNew) { // Consider only new edges for flipping
            VertexIter v1 = e->halfedge()->vertex();
            VertexIter v2 = e->halfedge()->twin()->vertex();

            if ((v1->isNew && !v2->isNew) || (!v1->isNew && v2->isNew)) {
                mesh.flipEdge(e);
            }
        }
    }

    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
      v->position = v->newPosition; // Update position
      v->isNew = false; // Reset isNew for all vertices
    }

  }
}
