#ifndef BEZIER_H
#define BEZIER_H

#include <iostream>
#include <functional>
#include <list>
#include <cmath>
#include <vector>
#include <cstdarg>

namespace bezier
{
    namespace types
    {
        using real_t = double;
        using node_index_t = unsigned long;

        class point_2d
        {
        public:
            point_2d() = delete;
            point_2d(real_t, real_t);

            const real_t X, Y;

            bool operator==(const point_2d &p) const {
                return this->X == p.X && this->Y == p.Y;
            }
        };

        inline point_2d::point_2d(real_t x, real_t y) : X(x), Y(y) {}

        inline std::ostream &operator<<(std::ostream &out, const point_2d &point)
        {
            out << "(" << point.X << ", " << point.Y << ")";
            return out;
        }

        inline point_2d operator+(const point_2d &lhs, const point_2d &rhs)
        {
            point_2d new_point(lhs.X + rhs.X, lhs.Y + rhs.Y);
            return new_point;
        }

        inline point_2d operator*(const real_t &scalar, const point_2d &point)
        {
            point_2d new_point(scalar * point.X, scalar * point.Y);
            return new_point;
        }

        inline point_2d operator*(const point_2d &point,const real_t &scalar)
        {
            return scalar * point;
        }

        using my_fun = std::function<const point_2d(node_index_t)>;
    } // namespace types

    namespace constants
    {
        const int NUM_OF_CUBIC_BEZIER_NODES = 4;
        const types::real_t ARC = 4 * (sqrt(2) - 1) / 3;
    } // namespace constants

    namespace detail 
    {
        // Checking if segment is viable
        inline std::function<bool(types::my_fun, types::node_index_t)> isViableSegment =
            [](types::my_fun fun, types::node_index_t n) {
                try {
                    return (fun(n).X  || true);
                }
                catch (const std::out_of_range &oor) {
                    return false;
                }
            };

        // Gets nth element from list
        template <typename T>
        inline T getNth(typename std::list<T>::const_iterator list, std::size_t size, types::node_index_t n){
            return 
                (n >= size) ? throw std::out_of_range("a curve node index is out of range") : 
                (n == 0) ? *list : getNth<T>(++list, size, n - 1);
        }

        // Creates function that represents curve, takes list of segments
        inline std::function<types::my_fun(std::list<types::my_fun>)> curve =
            [](std::list<types::my_fun> list) {
                return [list](types::node_index_t n) {
                    return getNth<types::my_fun>(list.begin(), list.size(), n / constants::NUM_OF_CUBIC_BEZIER_NODES)(n % constants::NUM_OF_CUBIC_BEZIER_NODES);
                };
            };

        // Creates function that represents segment, takes list of points
        inline  std::function<types::my_fun(std::list<std::function<const types::point_2d()>>)> segment =
            [](std::list<std::function<const types::point_2d()>> list) {
                return [list](types::node_index_t n) {
                    return getNth<std::function<const types::point_2d()>>(list.begin(), list.size(), n)();
                };
            };

        // Creates function that represents point, takes coordinates
        inline std::function<std::function<const types::point_2d()>(types::real_t, types::real_t)> point = 
            [](types::real_t x, types::real_t y) { return [x, y]() { return types::point_2d(x, y); }; }; 

        // Move

        // Copies segment, if certain node is found, it is modified
        inline std::function<types::my_fun(types::my_fun, types::node_index_t, types::real_t, types::real_t, types::node_index_t)> changeOneInSegment =
            [](types::my_fun fun, types::node_index_t n, types::real_t x, types::real_t y, types::node_index_t want) {
                return segment(std::list{(want == n) ? point(fun(n).X + x, fun(n).Y + y) : point(fun(n).X, fun(n ).Y),
                                         (want == n + 1) ? point(fun(n + 1).X + x, fun(n + 1).Y + y) : point(fun(n + 1).X, fun(n + 1).Y),
                                         (want == n + 2) ? point(fun(n + 2).X + x, fun(n + 2).Y + y) : point(fun(n + 2).X, fun(n + 2).Y),
                                         (want == n + 3) ? point(fun(n + 3).X + x, fun(n + 3).Y + y) : point(fun(n + 3).X, fun(n + 3).Y)});
            };

        // Adds old segments to return list, if certain node is found in segment, it is modified  
        inline std::function<std::list<types::my_fun>(std::list<types::my_fun> &, types::my_fun, types::node_index_t, types::real_t, types::real_t, types::node_index_t)> getCurveMove =
            [](std::list<types::my_fun> &list, types::my_fun fun, types::node_index_t n, types::real_t x, types::real_t y, types::node_index_t want) {
                (isViableSegment(fun, n) == 1) ? list.push_back(changeOneInSegment(fun, n, x, y, want)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveMove(list, fun, n + constants::NUM_OF_CUBIC_BEZIER_NODES, x, y, want) : 
                       (want < n) ? list : throw std::out_of_range("a curve node index is out of range");
            };

        // Translate

        // Transforms all nodes in segment with values x and y. 
        inline std::function<types::my_fun(types::my_fun, types::node_index_t, types::real_t, types::real_t)> changeInSegment =
            [](types::my_fun fun, types::node_index_t n, types::real_t x, types::real_t y) {
                return segment(std::list{point(fun(n).X + x, fun(n).Y + y), point(fun(n + 1).X + x, fun(n + 1).Y + y),
                                         point(fun(n + 2).X + x, fun(n + 2).Y + y), point(fun(n + 3).X + x, fun(n + 3).Y + y)});
            };

        //  Adds transformed segments to return list 
        inline std::function<std::list<types::my_fun>(std::list<types::my_fun> &, types::my_fun, types::node_index_t, types::real_t, types::real_t)> getCurveTranslate =
            [](std::list<types::my_fun> &list, types::my_fun fun, types::node_index_t n, types::real_t x, types::real_t y) {
                (isViableSegment(fun, n) == 1) ? list.push_back(changeInSegment(fun, n, x, y)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveTranslate(list, fun, n + constants::NUM_OF_CUBIC_BEZIER_NODES, x, y) : list;
            };

        //Scale 
       
        inline std::function<types::my_fun(types::my_fun, types::node_index_t, types::real_t, types::real_t)> scaleSegment =
            [](types::my_fun fun, types::node_index_t n, types::real_t x, types::real_t y) {
                return segment(std::list{point(fun(n).X * x, fun(n).Y * y), point(fun(n + 1).X * x, fun(n + 1).Y * y),
                                         point(fun(n + 2).X * x, fun(n + 2).Y * y), point(fun(n + 3).X * x, fun(n + 3).Y * y)});
            };

        // Scales all nodes in segment with values x and y. 
        inline std::function<std::list<types::my_fun>(std::list<types::my_fun> &, types::my_fun, types::node_index_t, types::real_t, types::real_t)> getCurveScale =
            [](std::list<types::my_fun> &list, types::my_fun fun, types::node_index_t n, types::real_t x, types::real_t y) {
                (isViableSegment(fun, n) == 1) ? list.push_back(scaleSegment(fun, n, x, y)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveScale(list, fun, n + constants::NUM_OF_CUBIC_BEZIER_NODES, x, y) : list;
            };

        // Rotate

       // Rotates all nodes in segment by angle a. 
        inline std::function<types::my_fun(types::my_fun, types::node_index_t, unsigned long)> rotateSegment =
            [](types::my_fun fun, types::node_index_t n, unsigned long a) {
                return segment(std::list{point(fun(n).X * cos(a * M_PI / 180) + (-1 * fun(n).Y * sin(a * M_PI / 180)), 
                                               fun(n).Y * cos(a * M_PI / 180) + fun(n).X * sin(a * M_PI / 180)),
                                         point(fun(n + 1).X * cos(a * M_PI / 180) + (-1 * fun(n + 1).Y * sin(a * M_PI / 180)), 
                                               fun(n + 1).Y * cos(a * M_PI / 180) + fun(n + 1).X * sin(a * M_PI / 180)),
                                         point(fun(n + 2).X * cos(a * M_PI / 180) + (-1 * fun(n + 2).Y * sin(a * M_PI / 180)), 
                                               fun(n + 2).Y * cos(a * M_PI / 180) + fun(n + 2).X * sin(a * M_PI / 180)),
                                         point(fun(n + 3).X * cos(a * M_PI / 180) + (-1 * fun(n + 3).Y * sin(a * M_PI / 180)) , 
                                               fun(n + 3).Y * cos(a * M_PI / 180) + fun(n + 3).X * sin(a * M_PI / 180))});
            };

        // Adds rotated segments to return list 
        inline std::function<std::list<types::my_fun>(std::list<types::my_fun> &, types::my_fun, types::node_index_t, unsigned long a)> getCurveRotate =
            [](std::list<types::my_fun> &list, types::my_fun fun, types::node_index_t n, unsigned long a) {
                (isViableSegment(fun, n) == 1) ? list.push_back(rotateSegment(fun, n, a)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveRotate(list, fun, n + constants::NUM_OF_CUBIC_BEZIER_NODES , a) : list;
            };

        // Concatenate

        //Copies segment
        inline std::function<types::my_fun(types::my_fun, types::node_index_t)> concatSegment =
            [](types::my_fun fun, types::node_index_t n) {
                return segment(std::list{point(fun(n).X , fun(n).Y),
                                         point(fun(n + 1).X , fun(n + 1).Y),
                                         point(fun(n + 2).X , fun(n + 2).Y),
                                         point(fun(n + 3).X , fun(n + 3).Y)});
            };

        // Adds copied segments to return list 
        inline std::function<std::list<types::my_fun>(std::list<types::my_fun> &, types::my_fun, types::node_index_t)> getCurveConcat =
            [](std::list<types::my_fun> &list, types::my_fun fun, types::node_index_t n) {
                (isViableSegment(fun, n) == 1) ? list.push_back(concatSegment(fun, n)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveConcat(list, fun, n + constants::NUM_OF_CUBIC_BEZIER_NODES) : list;
            };

        // Gets first segment 
        inline std::function<std::list<types::my_fun>(std::list<types::my_fun> &, types::my_fun)> getFirstSegmentConcat =
            [](std::list<types::my_fun> &list, types::my_fun fun) {
                (isViableSegment(fun, 0) == 1) ? list.push_back(concatSegment(fun, 0)) : (void)NULL;
                return list;
            };

        // Bez

        // Creates function that anwsers queries about bezier curve
        inline std::function<std::function<types::point_2d(double)>(std::list<types::point_2d>, int, int)> Bez = 
            [](std::list<types::point_2d> p, int deep, types::node_index_t pos) {
                return [p, deep, pos](double t) {
                    return (deep != 1) ? 
                    ((1 - t) * Bez(p, deep - 1, pos)(t) + t * Bez(p, deep - 1, pos + 1)(t)) : 
                    (1 - t) * getNth<types::point_2d>(p.begin(), p.size(), pos) + t * getNth<types::point_2d>(p.begin(), p.size(), (pos + 1));
                };
            };
    }

    // Generating base functions

    inline std::function<types::my_fun()> Cup = []() { 
        return detail::curve(std::list{detail::segment(std::list{detail::point(-1, 1), detail::point(-1, -1), 
                                                                 detail::point(1, -1), detail::point(1, 1)})}); };

    inline std::function<types::my_fun()> Cap = []() { 
        return detail::curve(std::list{detail::segment(std::list{detail::point(-1, -1), detail::point(-1, 1), 
                                                                 detail::point(1, 1), detail::point(1, -1)})});};

    inline std::function<types::my_fun()> ConvexArc = []() { 
        return detail::curve(std::list{detail::segment(std::list{detail::point(0, 1), detail::point(constants::ARC, 1), 
                                                                 detail::point(1, constants::ARC), detail::point(1, 0)})});};

    inline std::function<types::my_fun()> ConcaveArc = []() { 
        return detail::curve(std::list{detail::segment(std::list{detail::point(0, 1), detail::point(0, 1 - constants::ARC), 
                                                                 detail::point(1 - constants::ARC, 0), detail::point(1, 0)})});};

    inline std::function<types::my_fun(types::point_2d, types::point_2d)> LineSegment = 
        [](types::point_2d p, types::point_2d q) { 
        return detail::curve(std::list{detail::segment(std::list{detail::point(p.X, p.Y), detail::point(p.X, p.Y), 
                                                                 detail::point(q.X, q.Y), detail::point(q.X, q.Y)})});};

    // Move

    // Creating new function with one diffent value based on given function
    inline std::function<types::my_fun(types::my_fun, types::node_index_t, types::real_t, types::real_t)> MovePoint =
        [](types::my_fun fun, types::node_index_t want, types::real_t x, types::real_t y) {
            auto empty = std::list<types::my_fun>();
            return detail::curve(detail::getCurveMove(empty, fun, 0, x, y, want));
        };

    // Translate

    // Creating new translated function based on given function
    inline std::function<types::my_fun(types::my_fun, types::real_t, types::real_t)> Translate =
        [](types::my_fun fun, types::real_t x, types::real_t y) {
            auto empty = std::list<types::my_fun>();
            return detail::curve(detail::getCurveTranslate(empty, fun, 0, x, y));
        };

     // Scale

    // Creating new scaled function based on given function
    inline std::function<types::my_fun(types::my_fun, types::real_t, types::real_t)> Scale =
        [](types::my_fun fun, types::real_t x, types::real_t y) {
            auto empty = std::list<types::my_fun>();
            return detail::curve(detail::getCurveScale(empty, fun, 0, x, y));
        };

    // Rotate

    // Creating new rotated function based on given function
    inline std::function<types::my_fun(types::my_fun, unsigned long)> Rotate =
        [](types::my_fun fun, unsigned long a) {
            auto empty = std::list<types::my_fun>();
            return detail::curve(detail::getCurveRotate(empty, fun, 0, a));
        };

    // Concatenate

    // Creating new function based on given functions
    inline types::my_fun Concatenate (const types::my_fun& f1, const types::my_fun& f2) {
            auto empty1 = std::list<types::my_fun>(), empty2 = std::list<types::my_fun>();
            (detail::getFirstSegmentConcat(empty1, f1).splice(empty1.cend(), detail::getCurveConcat(empty2, f2, 0)));
            return detail::curve(empty1);
        };

    // Creating new function based on given functions
    template <typename ... Args> 
    inline types::my_fun Concatenate(const types::my_fun& f1, const types::my_fun& f2, Args ...funs) {
            return Concatenate(f1, Concatenate(f2, funs...));
        };


    class P3CurvePlotter
    {
        public:
            explicit P3CurvePlotter(const types::my_fun &, types::node_index_t n = 1, types::node_index_t res = 80);
            types::point_2d operator()(const types::my_fun &, types::real_t, types::node_index_t) const;
            void Print(std::ostream &s = std::cout, char fb = '*', char bg = ' ') const;
        private:
            std::vector<std::vector<bool>> screen;
            std::vector<std::vector<bool>> createScreen (const types::my_fun &, types::node_index_t, types::real_t);
            std::vector<std::vector<bool>> createScreenPom (const types::my_fun &f, types::node_index_t, types::real_t, 
                                                            std::vector<std::vector<bool>>, types::real_t, types::real_t);
    };

    inline P3CurvePlotter::P3CurvePlotter(const types::my_fun &f, types::node_index_t n, types::node_index_t res) : screen(createScreen(f, n-1, res)){}

    inline void P3CurvePlotter::Print(std::ostream& s, char fb, char bg) const {
        std::for_each(screen.begin(), screen.end(), [&](const auto &list){
            std::for_each(list.begin(), list.end(), [&](bool toPrint) {
                s << (toPrint ? fb : bg);
            });
            s << std::endl;
        });
    };

    inline types::point_2d P3CurvePlotter::operator()(const types::my_fun &f, types::real_t t, types::node_index_t n) const {
        auto pos = n  * constants::NUM_OF_CUBIC_BEZIER_NODES;
        return detail::Bez(std::list{f(pos), f(pos + 1), f(pos + 2), f(pos + 3)}, 3, 0)(t);
    };

    inline std::vector<std::vector<bool>> P3CurvePlotter::createScreenPom (const types::my_fun &f, types::node_index_t n, types::real_t res, 
                                                                           std::vector<std::vector<bool>> list, types::real_t t, types::real_t seg) {
        auto step = n / res / res;
        auto pos = seg * constants::NUM_OF_CUBIC_BEZIER_NODES;
        auto point = detail::Bez(std::list{f(pos), f(pos + 1), f(pos + 2), f(pos + 3)}, 3, 0)(t);
        auto mainList = list.begin();
        auto subList = (*mainList).begin();

        (std::round(point.X) >= 0 && std::round(point.X) < res &&
         std::round(point.Y) >= 0 && std::round(point.Y) < res)
            ? (mainList = list.begin(),
               std::advance(mainList, std::round(point.Y)),
               subList = (*mainList).begin(),
               std::advance(subList, std::round(point.X)),
               *subList = true)
            : *subList = *subList;

        (t + step <= 1) ? (t = t + step) : (t = 0, seg = seg + 1);
        return seg > n ? list : createScreenPom(f, n, res, list, t, seg);
    };

    inline std::vector<std::vector<bool>> P3CurvePlotter::createScreen (const types::my_fun &f, types::node_index_t n, types::real_t res) {
        std::vector<std::vector<bool>>newList;
        newList.resize(res);
        std::for_each(newList.begin(), newList.end(), [res](auto &list) { list.resize(res); });
        auto scale = (res - 1) / 2;

        return createScreenPom(Translate(Scale(f, scale, -scale), scale, scale), n, res, newList, 0, 0);
    };

} // namespace bezier

#endif // BEZIER_H
