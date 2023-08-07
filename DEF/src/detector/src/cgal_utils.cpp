// /* ----------------------------------------------------------------------------
//  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
//  * Massachusetts Institute of Technology
//  * All Rights Reserved
//  * Authors: Jesus Tordesillas, et al.
//  * See LICENSE file for the license information
//  * -------------------------------------------------------------------------- */

// #include "detector/cgal_utils.hpp"
// #include <CGAL/convex_hull_3.h>
// #include <CGAL/Triangulation_3.h>
// #include <chrono>

// // Convert several polyhedra to a vector that contains all the edges of all these polyhedra
// Edges vectorGCALPol2edges(const ConvexHullsOfCurves& convexHulls)
// {
//   // See example here:
//   // http://cgal-discuss.949826.n4.nabble.com/Take-the-triangles-of-a-polyhedron-td4460275.html

//   // Other related questions:
//   // https://doc.cgal.org/5.0/Triangulation_3/Triangulation_3_2for_loop_8cpp-example.html
//   // https://stackoverflow.com/questions/4837179/getting-a-vertex-handle-from-an-edge-iterator

//   Edges all_edges;

//   for (int index_curve = 0; index_curve < convexHulls.size(); index_curve++)  // for each curve
//   {
//     for (int i = 0; i < convexHulls[index_curve].size(); i++)  // for each interval along the curve
//     {
//       CGAL_Polyhedron_3 poly = convexHulls[index_curve][i];

//       for (CGAL_Polyhedron_3::Edge_iterator w = poly.edges_begin(); w != poly.edges_end();
//            ++w)  // for all the edges of that polyhedron
//       {
//         // std::cout << "First Vertex of the edge" << w->opposite()->vertex()->point() << std::endl;
//         // std::cout << "Second Vertex of the edge" << w->vertex()->point() << std::endl;

//         Eigen::Vector3d vertex1(w->opposite()->vertex()->point().x(), w->opposite()->vertex()->point().y(),
//                                 w->opposite()->vertex()->point().z());

//         Eigen::Vector3d vertex2(w->vertex()->point().x(), w->vertex()->point().y(), w->vertex()->point().z());

//         std::pair<Eigen::Vector3d, Eigen::Vector3d> edge;
//         edge.first = vertex1;
//         edge.second = vertex2;

//         all_edges.push_back(edge);
//       }
//     }
//   }

//   return all_edges; 
// }

// vec_E<Polyhedron<3>> vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves)
// {
//   vec_E<Polyhedron<3>> vector_of_polyhedron_jps;

//   for (auto convex_hulls_of_curve : convex_hulls_of_curves)
//   {
//     for (auto polyhedron_i : convex_hulls_of_curve)
//     {
//       vec_E<Hyperplane<3>> hyperplanes;
//       for (auto it = polyhedron_i.planes_begin(); it != polyhedron_i.planes_end(); it++)
//       {
//         Vector_3 n = it->orthogonal_vector();
//         Point_3 p = it->point();
//         hyperplanes.push_back(
//             Hyperplane<3>(Eigen::Vector3d(p.x(), p.y(), p.z()), Eigen::Vector3d(n.x(), n.y(), n.z())));
//       }
//       Polyhedron<3> polyhedron_jps(hyperplanes);
//       vector_of_polyhedron_jps.push_back(polyhedron_jps);
//       // std::cout << red << bold << "Size=" << vector_of_polyhedron_jps.size() << reset << std::endl;
//     }
//   }
//   return vector_of_polyhedron_jps;
// }

// CGAL_Polyhedron_3 convexHullOfPoints(const std::vector<Point_3>& points)
// {
//   CGAL::Object ch_object;

//   CGAL::convex_hull_3(points.begin(), points.end(), ch_object);
//   // std::cout << "convexHullCgal Computed!" << std::endl;

//   CGAL_Polyhedron_3 poly = *CGAL::object_cast<CGAL_Polyhedron_3>(&ch_object);

//   std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(),
//                  Plane_equation());  // Compute the planes

//   return poly;
// }



// ConvexHullsOfCurve_Std singlevectorGCALPol2vectorStdEigen(ConvexHullsOfCurve& convexHulls)
// {
//     auto start_time = std::chrono::high_resolution_clock::now();
//     ConvexHullsOfCurve_Std convexHulls_of_curve_std;
//     for (int i = 0; i < convexHulls.size(); i++)  // for each interval along the curve
//     {
//       CGAL_Polyhedron_3 poly = convexHulls[i];

//       Polyhedron_Std convexHull_std(3,
//                                         poly.size_of_vertices());  // poly.size_of_vertices() is the number of vertexes
//       int j = 0;
//       for (CGAL_Polyhedron_3::Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
//       {
//         Eigen::Vector3d vertex(v->point().x(), v->point().y(), v->point().z());
//         convexHull_std.col(j) = vertex;
//         j = j + 1;
//       }

//       convexHulls_of_curve_std.push_back(convexHull_std);
//     }
//     auto end_time = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//     std::cout << "执行时间ghull: " << duration.count() << " wei秒" << std::endl;

//     return convexHulls_of_curve_std;
// }


// ConvexHullsOfCurves_Std vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls)
// {
//   ConvexHullsOfCurves_Std convexHulls_of_curves_std;

//   // std::cout << "convexHulls.size()= " << convexHulls.size() << std::endl;

//   for (int index_curve = 0; index_curve < convexHulls.size(); index_curve++)  // for each curve
//   {
//     ConvexHullsOfCurve_Std convexHulls_of_curve_std;

//     for (int i = 0; i < convexHulls[index_curve].size(); i++)  // for each interval along the curve
//     {
//       CGAL_Polyhedron_3 poly = convexHulls[index_curve][i];

//       Polyhedron_Std convexHull_std(3,
//                                         poly.size_of_vertices());  // poly.size_of_vertices() is the number of vertexes
//       // std::vector<Eigen::Vector3d> convexHull_std;
//       int j = 0;
//       for (CGAL_Polyhedron_3::Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
//       {
//         Eigen::Vector3d vertex(v->point().x(), v->point().y(), v->point().z());
//         convexHull_std.col(j) = vertex;
//         // convexHull_std.push_back(vertex);
//         j = j + 1;
//         // std::cout << v->point() << std::endl;
//       }

//       convexHulls_of_curve_std.push_back(convexHull_std);
//     }

//     convexHulls_of_curves_std.push_back(convexHulls_of_curve_std);
//   }
//   // std::cout << "convexHulls_of_curves_std.size()= " << convexHulls_of_curves_std.size() << std::endl;

//   return convexHulls_of_curves_std;
// }


