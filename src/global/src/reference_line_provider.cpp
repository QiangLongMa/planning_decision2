#include"reference_line_provider.h"

AnchorPoint ReferenceLineProvider::GetAnchorPoint(Eigen::MatrixXd &RawglobalPath,Eigen::VectorXd col){
    AnchorPoint anchor;
    anchor.longitudinal_bound =longitudinal_boundary_bound;
    const double adc_width = vehicle_width;
    anchor.s = col(6);
    anchor.lateral_bound = min_lateral_boundary_bound;
    anchor.x = col(0);
    anchor.y = col(1);
    //std::cout<<"anchor.s : "<<anchor.s << " anchor.x:"<< anchor.x<<"  anchor.y: "<< anchor.y<<std::endl;
    return anchor;
}
void ReferenceLineProvider::GetAnchorPoints(Eigen::MatrixXd &RawglobalPath,std::vector<AnchorPoint> *anchor_points){
    const double interval =max_constraint_interval;

    std::vector<double> anchor_s;
    for (size_t i = 0; i < RawglobalPath.cols(); ++i)
    {
        AnchorPoint anchor = GetAnchorPoint(RawglobalPath, RawglobalPath.col(i));
        anchor_points->emplace_back(anchor);
    }
    //起点和终点是强约束
    anchor_points->front().longitudinal_bound = 1e-6;
    anchor_points->front().lateral_bound = 1e-6;
    anchor_points->front().enforced = true;
    anchor_points->back().longitudinal_bound = 1e-6;
    anchor_points->back().lateral_bound = 1e-6;
    anchor_points->back().enforced = true;
}
bool ReferenceLineProvider::SmoothReferenceLine(Eigen::MatrixXd &RawglobalPath,Eigen::MatrixXd &NewglobalPath){

    if (RawglobalPath.cols()==0){
        return false;
    }
    if (!FLAGS_enable_smooth_reference_line) {
        NewglobalPath = RawglobalPath;
        return true;
    }
    // generate anchor points:
    std::vector<AnchorPoint> anchor_points;
    GetAnchorPoints(RawglobalPath,&anchor_points);

    smooth_.SetAnchorPoints(anchor_points);
     if (!smooth_.Smooth(RawglobalPath, NewglobalPath)) {
    std::cout << "Failed to smooth reference line with anchor points"<<std::endl;
    return false;
    }
  return true;
}