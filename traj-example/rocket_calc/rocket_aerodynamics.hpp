#pragma once
#include "../libs/math/interpolation.hpp"
#include <cmath>

// std::vector<std::vector<double>> C_x_1st_stage = {
//     {0.2138555, 0.243990914, 0.303490842, 0.375807735, 0.487680231, 0.482916113, 0.468186962, 0.433690853, 0.344686467, 0.304375399, 0.213644817},
//     {0.202256411, 0.246504968, 0.3147496, 0.380112292, 0.488493245, 0.486911119, 0.472555969, 0.439205487, 0.351445339, 0.312198293, 0.227116528},
//     {0.183748401, 0.256359971, 0.315301665, 0.393855373, 0.490627619, 0.489809108, 0.47730773, 0.443740107, 0.36067905, 0.324945705, 0.231234062},
//     {0.158686096, 0.2675387, 0.324140765, 0.421979603, 0.491384252, 0.497278305, 0.485509709, 0.446580569, 0.376521089, 0.330132141, 0.23542179},
//     {0.135890173, 0.284241566, 0.341515561, 0.431581802, 0.498402035, 0.505598905, 0.488557029, 0.454078888, 0.391951982, 0.338093829, 0.243596303}};

const std::vector<double> Mach_grid_1st_stage = {0.0, 0.4, 0.7, 0.8, 0.9, 1.2, 1.4, 1.6, 2.0, 3.0, 4.0, 8.0};
const std::vector<double> aoa_grid_1st_stage = {0.0, 3.0, 5.0, 7.0, 10.0};

const std::vector<double> Mach_grid_2nd_stage = {2.0, 4.0, 8.0, 12.0};
const std::vector<double> aoa_grid_2nd_stage = {0.0, 3.0, 5.0, 7.0, 10.0};

const std::vector<double> Mach_grid_3rd_stage = {0.4, 0.6, 0.8, 1.2, 1.4, 1.6, 2.0, 3.0, 4.0, 8.0, 12.0, 16.0};
const std::vector<double> aoa_grid_3rd_stage = {0.0, 3.0, 7.0, 11.0, 15.0};

namespace detail
{
    const math::points C_x_1st_stage_aoa0 = {Mach_grid_1st_stage,
                                             {0.0, 0.2138555, 0.243990914, 0.303490842, 0.375807735, 0.487680231, 0.482916113, 0.468186962, 0.433690853, 0.344686467, 0.304375399, 0.213644817}};
    const math::points C_x_1st_stage_aoa3 = {Mach_grid_1st_stage,
                                             {0.0, 0.202256411, 0.246504968, 0.3147496, 0.380112292, 0.488493245, 0.486911119, 0.472555969, 0.439205487, 0.351445339, 0.312198293, 0.227116528}};
    const math::points C_x_1st_stage_aoa5 = {Mach_grid_1st_stage,
                                             {0.0, 0.183748401, 0.256359971, 0.315301665, 0.393855373, 0.490627619, 0.489809108, 0.47730773, 0.443740107, 0.36067905, 0.324945705, 0.231234062}};
    const math::points C_x_1st_stage_aoa7 = {Mach_grid_1st_stage,
                                             {0.0, 0.158686096, 0.2675387, 0.324140765, 0.421979603, 0.491384252, 0.497278305, 0.485509709, 0.446580569, 0.376521089, 0.330132141, 0.23542179}};
    const math::points C_x_1st_stage_aoa10 = {Mach_grid_1st_stage,
                                              {0.0, 0.135890173, 0.284241566, 0.341515561, 0.431581802, 0.498402035, 0.505598905, 0.488557029, 0.454078888, 0.391951982, 0.338093829, 0.243596303}};

    // auto C_x_1st_stage_aoa0_interpolant = CubicSpline(C_x_1st_stage_aoa0);
    // auto C_x_1st_stage_aoa3_interpolant = CubicSpline(C_x_1st_stage_aoa3);
    // auto C_x_1st_stage_aoa5_interpolant = CubicSpline(C_x_1st_stage_aoa5);
    // auto C_x_1st_stage_aoa7_interpolant = CubicSpline(C_x_1st_stage_aoa7);
    // auto C_x_1st_stage_aoa10_interpolant = CubicSpline(C_x_1st_stage_aoa10);

    // std::vector<std::vector<double>>
    //     C_y_1st_stage = {
    //         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    //         {0.067529645, 0.093783925, 0.117191339, 0.11091527, 0.105485603, 0.108632778, 0.109390014, 0.110934465, 0.111956695, 0.11215483, 0.112171635},
    //         {0.126347967, 0.175469724, 0.219264995, 0.207522469, 0.197363564, 0.203251928, 0.204668716, 0.22308921, 0.225380024, 0.232512663, 0.237142053},
    //         {0.178729307, 0.248215963, 0.310167877, 0.293557135, 0.27918655, 0.287516112, 0.289520273, 0.315577534, 0.318818074, 0.328907763, 0.335456406},
    //         {0.30938043, 0.429661831, 0.536900595, 0.519394929, 0.502358184, 0.511294959, 0.531757114, 0.546264712, 0.551874086, 0.569339338, 0.580675038}};

    const math::points C_y_1st_stage_aoa0 = {Mach_grid_1st_stage,
                                             {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    const math::points C_y_1st_stage_aoa3 = {Mach_grid_1st_stage,
                                             {0, 0.067529645, 0.093783925, 0.117191339, 0.11091527, 0.105485603, 0.108632778, 0.109390014, 0.110934465, 0.111956695, 0.11215483, 0.112171635}};
    const math::points C_y_1st_stage_aoa5 = {Mach_grid_1st_stage,
                                             {0, 0.126347967, 0.175469724, 0.219264995, 0.207522469, 0.197363564, 0.203251928, 0.204668716, 0.22308921, 0.225380024, 0.232512663, 0.237142053}};
    const math::points C_y_1st_stage_aoa7 = {Mach_grid_1st_stage,
                                             {0, 0.178729307, 0.248215963, 0.310167877, 0.293557135, 0.27918655, 0.287516112, 0.289520273, 0.315577534, 0.318818074, 0.328907763, 0.335456406}};
    const math::points C_y_1st_stage_aoa10 = {Mach_grid_1st_stage,
                                              {0, 0.30938043, 0.429661831, 0.536900595, 0.519394929, 0.502358184, 0.511294959, 0.531757114, 0.546264712, 0.551874086, 0.569339338, 0.580675038}};

    // auto C_y_1st_stage_aoa0_interpolant = CubicSpline(C_y_1st_stage_aoa0);
    // auto C_y_1st_stage_aoa3_interpolant = CubicSpline(C_y_1st_stage_aoa3);
    // auto C_y_1st_stage_aoa5_interpolant = CubicSpline(C_y_1st_stage_aoa5);
    // auto C_y_1st_stage_aoa7_interpolant = CubicSpline(C_y_1st_stage_aoa7);
    // auto C_y_1st_stage_aoa10_interpolant  CubicSpline(C_y_1st_stage_aoa10);

    // std::vector<std::vector<double>> C_x_2nd_stage = {
    //     {0.392150461, 0.279961371, 0.225813158, 0.212599438},
    //     {0.409541221, 0.282105638, 0.231111128, 0.217587391},
    //     {0.422155812, 0.294690132, 0.249597846, 0.239996223},
    //     {0.437787936, 0.299011783, 0.260055315, 0.252821217},
    //     {0.442589993, 0.302918167, 0.27617085, 0.273842337}};

    const math::points C_x_2nd_stage_aoa0 = {Mach_grid_2nd_stage,
                                             {0.392150461, 0.279961371, 0.225813158, 0.212599438}};
    const math::points C_x_2nd_stage_aoa3 = {Mach_grid_2nd_stage,
                                             {0.409541221, 0.282105638, 0.231111128, 0.217587391}};
    const math::points C_x_2nd_stage_aoa5 = {Mach_grid_2nd_stage,
                                             {0.422155812, 0.294690132, 0.249597846, 0.239996223}};
    const math::points C_x_2nd_stage_aoa7 = {Mach_grid_2nd_stage,
                                             {0.437787936, 0.299011783, 0.260055315, 0.252821217}};
    const math::points C_x_2nd_stage_aoa10 = {Mach_grid_2nd_stage,
                                              {0.442589993, 0.302918167, 0.27617085, 0.273842337}};

    // auto C_x_2nd_stage_aoa0_interpolant = CubicSpline(C_x_2nd_stage_aoa0);
    // auto C_x_2nd_stage_aoa3_interpolant = CubicSpline(C_x_2nd_stage_aoa3);
    // auto C_x_2nd_stage_aoa5_interpolant = CubicSpline(C_x_2nd_stage_aoa5);
    // auto C_x_2nd_stage_aoa7_interpolant = CubicSpline(C_x_2nd_stage_aoa7);
    // auto C_x_2nd_stage_aoa10_interpolant = CubicSpline(C_x_2nd_stage_aoa10);

    // std::vector<std::vector<double>> C_y_2nd_stage = {
    //     {0, 0, 0, 0},
    //     {0.259882284, 0.167153125, 0.139880581, 0.133517288},
    //     {0.446175611, 0.325665715, 0.271655146, 0.262858285},
    //     {0.641525771, 0.491535493, 0.413235264, 0.394366629},
    //     {0.781499141, 0.67584304, 0.581743401, 0.559184172}};

    const math::points C_y_2nd_stage_aoa0 = {Mach_grid_2nd_stage,
                                             {0, 0, 0, 0}};
    const math::points C_y_2nd_stage_aoa3 = {Mach_grid_2nd_stage,
                                             {0.259882284, 0.167153125, 0.139880581, 0.133517288}};
    const math::points C_y_2nd_stage_aoa5 = {Mach_grid_2nd_stage,
                                             {0.446175611, 0.325665715, 0.271655146, 0.262858285}};
    const math::points C_y_2nd_stage_aoa7 = {Mach_grid_2nd_stage,
                                             {0.641525771, 0.491535493, 0.413235264, 0.394366629}};
    const math::points C_y_2nd_stage_aoa10 = {Mach_grid_2nd_stage,
                                              {0.781499141, 0.67584304, 0.581743401, 0.559184172}};

    // auto C_y_2nd_stage_aoa0_interpolant = CubicSpline(C_y_2nd_stage_aoa0);
    // auto C_y_2nd_stage_aoa3_interpolant = CubicSpline(C_y_2nd_stage_aoa3);
    // auto C_y_2nd_stage_aoa5_interpolant = CubicSpline(C_y_2nd_stage_aoa5);
    // auto C_y_2nd_stage_aoa7_interpolant = CubicSpline(C_y_2nd_stage_aoa7);
    // auto C_y_2nd_stage_aoa10_interpolant= CubicSpline(C_y_2nd_stage_aoa10);

    // std::vector<std::vector<double>> m_z_2nd_stage = {
    //     {0, 0, 0, 0},
    //     {0.106545422, 0.070971876, 0.05927725, 0.055423198},
    //     {0.207486662, 0.141311457, 0.115408006, 0.109190883},
    //     {0.30863492, 0.203280284, 0.16021769, 0.150755323},
    //     {0.422336308, 0.297657746, 0.241912385, 0.229560277}};

    // std::vector<std::vector<double>> C_x_3rd_stage = {
    //     {0.253990284, 0.311041712, 0.34683456, 0.4512376, 0.470613182, 0.471973654, 0.437570055, 0.336334079, 0.324309541, 0.292616411, 0.277176731, 0.277529628},
    //     {0.309907283, 0.329691852, 0.350928069, 0.45952463, 0.478423805, 0.480451318, 0.447868706, 0.343290124, 0.327592684, 0.298778229, 0.28352523, 0.280339195},
    //     {0.321428069, 0.337755409, 0.386132113, 0.465358489, 0.48265311, 0.498188463, 0.458230366, 0.350705162, 0.34481005, 0.303243965, 0.294693147, 0.291381616},
    //     {0.248862442, 0.364321678, 0.430290167, 0.471146666, 0.485562007, 0.50604482, 0.462318938, 0.358643108, 0.34766785, 0.308871828, 0.300899904, 0.297518626},
    //     {0.259545004, 0.373887248, 0.451868454, 0.484731301, 0.49689667, 0.509295401, 0.46755093, 0.370974352, 0.360694699, 0.323225813, 0.315274702, 0.312077133}};

    const math::points C_x_3rd_stage_aoa0 = {Mach_grid_3rd_stage,
                                             {0.253990284, 0.311041712, 0.34683456, 0.4512376, 0.470613182, 0.471973654, 0.437570055, 0.336334079, 0.324309541, 0.292616411, 0.277176731, 0.277529628}};
    const math::points C_x_3rd_stage_aoa3 = {Mach_grid_3rd_stage,
                                             {0.309907283, 0.329691852, 0.350928069, 0.45952463, 0.478423805, 0.480451318, 0.447868706, 0.343290124, 0.327592684, 0.298778229, 0.28352523, 0.280339195}};
    const math::points C_x_3rd_stage_aoa7 = {Mach_grid_3rd_stage,
                                             {0.321428069, 0.337755409, 0.386132113, 0.465358489, 0.48265311, 0.498188463, 0.458230366, 0.350705162, 0.34481005, 0.303243965, 0.294693147, 0.291381616}};
    const math::points C_x_3rd_stage_aoa11 = {Mach_grid_3rd_stage,
                                              {0.248862442, 0.364321678, 0.430290167, 0.471146666, 0.485562007, 0.50604482, 0.462318938, 0.358643108, 0.34766785, 0.308871828, 0.300899904, 0.297518626}};
    const math::points C_x_3rd_stage_aoa15 = {Mach_grid_3rd_stage,
                                              {0.259545004, 0.373887248, 0.451868454, 0.484731301, 0.49689667, 0.509295401, 0.46755093, 0.370974352, 0.360694699, 0.323225813, 0.315274702, 0.312077133}};

    // auto C_x_3rd_stage_aoa0_interpolant = CubicSpline(C_x_3rd_stage_aoa0);
    // auto C_x_3rd_stage_aoa3_interpolant = CubicSpline(C_x_3rd_stage_aoa3);
    // auto C_x_3rd_stage_aoa7_interpolant = CubicSpline(C_x_3rd_stage_aoa7);
    // auto C_x_3rd_stage_aoa11_interpolant = CubicSpline(C_x_3rd_stage_aoa11);
    // auto C_x_3rd_stage_aoa15_interpolant = CubicSpline(C_x_3rd_stage_aoa15);

    // std::vector<std::vector<double>> C_y_3rd_stage = {
    //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    //     {0.147337398, 0.279056033, 0.328723769, 0.097858257, 0.085234454, 0.076063708, 0.072297787, 0.048889628, 0.05090605, 0.044442114, 0.043024407, 0.042540112},
    //     {0.374737635, 0.56701983, 0.519484318, 0.241567521, 0.20163842, 0.183942887, 0.162525637, 0.114733198, 0.118923145, 0.102617701, 0.099686712, 0.09856461},
    //     {0.620676261, 0.766715096, 0.655103906, 0.395912057, 0.311778274, 0.282052717, 0.246530037, 0.184173154, 0.18893928, 0.164948194, 0.159686335, 0.157888859},
    //     {0.895458481, 0.787994683, 0.75059721, 0.521612711, 0.446601828, 0.380999905, 0.336714312, 0.261019588, 0.261641706, 0.22925954, 0.222532818, 0.219738891}};

    const math::points C_y_3rd_stage_aoa0 = {Mach_grid_3rd_stage,
                                             {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    const math::points C_y_3rd_stage_aoa3 = {Mach_grid_3rd_stage,
                                             {0.147337398, 0.279056033, 0.328723769, 0.097858257, 0.085234454, 0.076063708, 0.072297787, 0.048889628, 0.05090605, 0.044442114, 0.043024407, 0.042540112}};
    const math::points C_y_3rd_stage_aoa7 = {Mach_grid_3rd_stage,
                                             {0.374737635, 0.56701983, 0.519484318, 0.241567521, 0.20163842, 0.183942887, 0.162525637, 0.114733198, 0.118923145, 0.102617701, 0.099686712, 0.09856461}};
    const math::points C_y_3rd_stage_aoa11 = {Mach_grid_3rd_stage,
                                              {0.620676261, 0.766715096, 0.655103906, 0.395912057, 0.311778274, 0.282052717, 0.246530037, 0.184173154, 0.18893928, 0.164948194, 0.159686335, 0.157888859}};
    const math::points C_y_3rd_stage_aoa15 = {Mach_grid_3rd_stage,
                                              {0.895458481, 0.787994683, 0.75059721, 0.521612711, 0.446601828, 0.380999905, 0.336714312, 0.261019588, 0.261641706, 0.22925954, 0.222532818, 0.219738891}};

    // auto C_y_3rd_stage_aoa0_interpolant = CubicSpline(C_y_3rd_stage_aoa0);
    // auto C_y_3rd_stage_aoa3_interpolant = CubicSpline(C_y_3rd_stage_aoa3);
    // auto C_y_3rd_stage_aoa7_interpolant = CubicSpline(C_y_3rd_stage_aoa7);
    // auto C_y_3rd_stage_aoa11_interpolant = CubicSpline(C_y_3rd_stage_aoa11);
    // auto C_y_3rd_stage_aoa15_interpolant = CubicSpline(C_y_3rd_stage_aoa15);

    // std::vector<std::vector<double>> m_z_3rd_stage = {
    //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    //     {0.016520372, -0.013189122, -0.036984858, -0.012748905, -0.010868115, -0.009825304, -0.014080323, -0.010583559, -0.004952241, -0.003706586, -0.003451741, -0.003311255},
    //     {0.023577215, -0.019582697, -0.03981462, -0.026597517, -0.021846961, -0.022568757, -0.030663697, -0.023411549, -0.013471144, -0.00968575, -0.00860592, -0.008270884},
    //     {0.019973002, -0.020357891, -0.041258662, -0.034700619, -0.034437463, -0.034041763, -0.040636748, -0.035912512, -0.020314976, -0.015403565, -0.01439577, -0.01384952},
    //     {0.02295803, -0.011807899, -0.023272427, -0.044576414, -0.044345196, -0.043478067, -0.056776931, -0.054507135, -0.033820301, -0.026113977, -0.024151822, -0.023375128}};

    const math::points m_z_3rd_stage_aoa0 = {Mach_grid_3rd_stage,
                                             {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    const math::points m_z_3rd_stage_aoa3 = {Mach_grid_3rd_stage,
                                             {0.016520372, -0.013189122, -0.036984858, -0.012748905, -0.010868115, -0.009825304, -0.014080323, -0.010583559, -0.004952241, -0.003706586, -0.003451741, -0.003311255}};
    const math::points m_z_3rd_stage_aoa7 = {Mach_grid_3rd_stage,
                                             {0.023577215, -0.019582697, -0.03981462, -0.026597517, -0.021846961, -0.022568757, -0.030663697, -0.023411549, -0.013471144, -0.00968575, -0.00860592, -0.008270884}};
    const math::points m_z_3rd_stage_aoa11 = {Mach_grid_3rd_stage,
                                              {0.019973002, -0.020357891, -0.041258662, -0.034700619, -0.034437463, -0.034041763, -0.040636748, -0.035912512, -0.020314976, -0.015403565, -0.01439577, -0.01384952}};
    const math::points m_z_3rd_stage_aoa15 = {Mach_grid_3rd_stage,
                                              {0.02295803, -0.011807899, -0.023272427, -0.044576414, -0.044345196, -0.043478067, -0.056776931, -0.054507135, -0.033820301, -0.026113977, -0.024151822, -0.023375128}};
};
// auto m_z_3rd_stage_aoa0_interpolant = CubicSpline(m_z_3rd_stage_aoa0);
// auto m_z_3rd_stage_aoa3_interpolant = CubicSpline(m_z_3rd_stage_aoa3);
// auto m_z_3rd_stage_aoa7_interpolant = CubicSpline(m_z_3rd_stage_aoa7);
// auto m_z_3rd_stage_aoa11_interpolant = CubicSpline(m_z_3rd_stage_aoa11);
// auto m_z_3rd_stage_aoa15_interpolant = CubicSpline(m_z_3rd_stage_aoa15);

std::vector<std::function<double(double)>> C_x_1st_stage_interpolants = {CubicSpline(detail::C_x_1st_stage_aoa0),
                                                                         CubicSpline(detail::C_x_1st_stage_aoa3),
                                                                         CubicSpline(detail::C_x_1st_stage_aoa5),
                                                                         CubicSpline(detail::C_x_1st_stage_aoa7),
                                                                         CubicSpline(detail::C_x_1st_stage_aoa10)};

std::vector<std::function<double(double)>> C_y_1st_stage_interpolants = {CubicSpline(detail::C_y_1st_stage_aoa0),
                                                                         CubicSpline(detail::C_y_1st_stage_aoa3),
                                                                         CubicSpline(detail::C_y_1st_stage_aoa5),
                                                                         CubicSpline(detail::C_y_1st_stage_aoa7),
                                                                         CubicSpline(detail::C_y_1st_stage_aoa10)};

std::vector<std::function<double(double)>> C_x_2nd_stage_interpolants = {CubicSpline(detail::C_x_2nd_stage_aoa0),
                                                                         CubicSpline(detail::C_x_2nd_stage_aoa3),
                                                                         CubicSpline(detail::C_x_2nd_stage_aoa5),
                                                                         CubicSpline(detail::C_x_2nd_stage_aoa7),
                                                                         CubicSpline(detail::C_x_2nd_stage_aoa10)};

std::vector<std::function<double(double)>> C_y_2nd_stage_interpolants = {CubicSpline(detail::C_y_2nd_stage_aoa0),
                                                                         CubicSpline(detail::C_y_2nd_stage_aoa3),
                                                                         CubicSpline(detail::C_y_2nd_stage_aoa5),
                                                                         CubicSpline(detail::C_y_2nd_stage_aoa7),
                                                                         CubicSpline(detail::C_y_2nd_stage_aoa10)};

std::vector<std::function<double(double)>> C_x_3rd_stage_interpolants = {CubicSpline(detail::C_x_3rd_stage_aoa0),
                                                                         CubicSpline(detail::C_x_3rd_stage_aoa3),
                                                                         CubicSpline(detail::C_x_3rd_stage_aoa7),
                                                                         CubicSpline(detail::C_x_3rd_stage_aoa11),
                                                                         CubicSpline(detail::C_x_3rd_stage_aoa15)};

std::vector<std::function<double(double)>> C_y_3rd_stage_interpolants = {CubicSpline(detail::C_y_3rd_stage_aoa0),
                                                                         CubicSpline(detail::C_y_3rd_stage_aoa3),
                                                                         CubicSpline(detail::C_y_3rd_stage_aoa7),
                                                                         CubicSpline(detail::C_y_3rd_stage_aoa11),
                                                                         CubicSpline(detail::C_y_3rd_stage_aoa15)};

std::vector<std::function<double(double)>> m_z_3rd_stage_interpolants = {CubicSpline(detail::m_z_3rd_stage_aoa0),
                                                                         CubicSpline(detail::m_z_3rd_stage_aoa3),
                                                                         CubicSpline(detail::m_z_3rd_stage_aoa7),
                                                                         CubicSpline(detail::m_z_3rd_stage_aoa11),
                                                                         CubicSpline(detail::m_z_3rd_stage_aoa15)};

// double C_x_1st_stage(double Mach, double aoa_deg)
// {

//     size_t len_aoa_grid = aoa_grid_1st_stage.size();

//     size_t len_Mach_grid = mach_grid_1st_stage.size();

//     if (aoa_deg < 0.0 || aoa_deg > 10.0)
//         throw std::out_of_range("C_x_1st_stage aoa out of range");

//     size_t mach_range;
//     for (size_t i = 0; i < mach_grid_1st_stage.size(); i++)
//     {
//         if (Mach >= mach_grid_1st_stage[i] && Mach < mach_grid_1st_stage[i + 1])
//         {
//             mach_range = i;
//             break;
//         }
//     }
//     size_t aoa_range;

//     for (size_t i = 0; i < aoa_grid_1st_stage.size(); i++)
//         if (aoa_deg >= aoa_grid_1st_stage[i] && aoa_deg < aoa_grid_1st_stage[i + 1])
//         {
//             aoa_range = i;
//             break;
//         }
//     if ((fabs(aoa_deg - aoa_grid_1st_stage[aoa_grid_1st_stage.size() - 1]) < 1e-6))
//         aoa_range = aoa_grid_1st_stage.size() - 2;
//     return linear_function_from2points(aoa_grid_1st_stage[aoa_range], aoa_grid_1st_stage[aoa_range + 1], C_x_1st_stage_interpolants[aoa_range](Mach), C_x_1st_stage_interpolants[aoa_range + 1](Mach))(aoa_deg);
// }

double aerodynamic_coeff(double Mach,
                         double aoa_deg,
                         std::vector<double> aoa_grid,
                         std::vector<double> Mach_grid,
                         std::vector<std::function<double(double)>> coeff)
{

    size_t len_aoa_grid = aoa_grid.size();

    size_t len_Mach_grid = Mach_grid.size();

    // if (aoa_deg < 0.0)
    // {
    //     // std::cerr << aoa_deg << " corrected aoa to be 0\n";
    //     aoa_deg = -1.0 * aoa_deg;
    // }
    aoa_deg = std::clamp(aoa_deg, aoa_grid[0], aoa_grid[aoa_grid.size() - 1]);

    
    if (aoa_deg < aoa_grid[0] || aoa_deg > aoa_grid[aoa_grid.size() - 1])
        throw std::out_of_range("aoa out of range");

    size_t mach_range = -1;
    for (size_t i = 0; i < Mach_grid.size(); i++)
        if (Mach >= Mach_grid[i] && Mach < Mach_grid[i + 1])
        {
            mach_range = i;
            break;
        }

    if (mach_range == -1)
    {
        //std::cerr << "Corrected Mach number to " << Mach_grid[Mach_grid.size() - 1] << " (was " << Mach << ")\n";
        Mach = Mach_grid[Mach_grid.size() - 1];
    }

    size_t aoa_range;
    for (size_t i = 0; i < aoa_grid.size(); i++)
        if (aoa_deg >= aoa_grid[i] && aoa_deg < aoa_grid[i + 1])
        {
            aoa_range = i;
            break;
        }

    if ((fabs(aoa_deg - aoa_grid[aoa_grid.size() - 1]) < 1e-6))
        aoa_range = aoa_grid.size() - 2;
    return math::linear_function_from2points(aoa_grid[aoa_range], aoa_grid[aoa_range + 1], coeff[aoa_range](Mach), coeff[aoa_range + 1](Mach))(aoa_deg);
}

auto C_x_1st_stage = [](double Mach, double aoa_deg)
{
    return aerodynamic_coeff(Mach, aoa_deg, aoa_grid_1st_stage, Mach_grid_1st_stage, C_x_1st_stage_interpolants);
};

auto C_y_1st_stage = [](double Mach, double aoa_deg)
{
    return aerodynamic_coeff(Mach, aoa_deg, aoa_grid_1st_stage, Mach_grid_1st_stage, C_y_1st_stage_interpolants);
};

auto C_x_2nd_stage = [](double Mach, double aoa_deg)
{
    return aerodynamic_coeff(Mach, aoa_deg, aoa_grid_2nd_stage, Mach_grid_2nd_stage, C_x_2nd_stage_interpolants);
};

auto C_y_2nd_stage = [](double Mach, double aoa_deg)
{
    return aerodynamic_coeff(Mach, aoa_deg, aoa_grid_2nd_stage, Mach_grid_2nd_stage, C_y_2nd_stage_interpolants);
};

auto C_x_3rd_stage = [](double Mach, double aoa_deg)
{
    return aerodynamic_coeff(Mach, aoa_deg, aoa_grid_3rd_stage, Mach_grid_3rd_stage, C_x_3rd_stage_interpolants);
};

auto C_y_3rd_stage = [](double Mach, double aoa_deg)
{
    return aerodynamic_coeff(Mach, aoa_deg, aoa_grid_3rd_stage, Mach_grid_3rd_stage, C_y_3rd_stage_interpolants);
};

auto m_z_3rd_stage = [](double Mach, double aoa_deg)
{
    return aerodynamic_coeff(Mach, aoa_deg, aoa_grid_3rd_stage, Mach_grid_3rd_stage, m_z_3rd_stage_interpolants);
};