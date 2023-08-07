/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */
#ifndef _CGAL_H_
#define _CGAL_H_
#pragma once
#include <Eigen/Dense>
// #include <CGAL/Polyhedron_3.h>
// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Convex_hull_traits_3.h>
// // #include <decomp_geometry/polyhedron.h>
// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/convex_hull_3.h>

// typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// typedef CGAL::Convex_hull_traits_3<K> Traits;
// typedef Traits::Polyhedron_3 CGAL_Polyhedron_3;   // 三维多面体类
// typedef K::Segment_3 Segment_3;                    // 三维线段
// typedef K::Plane_3 Plane_3;                        // 三维平面
// // define point creator
// typedef K::Point_3 Point_3;
// typedef K::Vector_3 Vector_3;
// typedef CGAL::Creator_uniform_3<double, Point_3> PointCreator;

// typedef std::vector<CGAL_Polyhedron_3> ConvexHullsOfCurve; //凸包曲线
// typedef std::vector<ConvexHullsOfCurve> ConvexHullsOfCurves;


typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Polyhedron_Std;   // 3 * dynamic 一个多面体
typedef std::vector<Polyhedron_Std> ConvexHullsOfCurve_Std;        // 凸包，多面体集合
typedef std::vector<ConvexHullsOfCurve_Std> ConvexHullsOfCurves_Std; // 多面体集合的集合
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Edge;          // 多面体的边
typedef std::vector<Edge> Edges;                                   // 多面体边的集合

// struct Plane_equation   // 三个点产生三角形平面方程
// {
//   template <class Facet>
//   typename Facet::Plane_3 operator()(Facet& f)
//   {
//     typename Facet::Halfedge_handle h = f.halfedge();
//     typedef typename Facet::Plane_3 Plane;
//     return Plane(h->vertex()->point(), h->next()->vertex()->point(), h->next()->next()->vertex()->point());
//   }
// };

// ConvexHullsOfCurves_Std vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls);   // // 多面体集合的集合

// vec_E<Polyhedron<3>> vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves); // 转化为多面体

// CGAL_Polyhedron_3 convexHullOfPoints(const std::vector<Point_3>& points);  // 点转化成多面体

// Edges vectorGCALPol2edges(const ConvexHullsOfCurves& convexHulls);  // 凸包转化为边

// ConvexHullsOfCurve_Std singlevectorGCALPol2vectorStdEigen(ConvexHullsOfCurve& convexHulls);


#endif