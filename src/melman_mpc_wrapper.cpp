#include <vehicle_controller/melman_mpc_wrapper.h>
#include <algorithm>

#include <ros/ros.h>

USING_NAMESPACE_ACADO

MelmanMpcWrapper::MelmanMpcWrapper(const MotionParameters &mp) : Q(4,4), r(4), mp(mp)
{
    setupODE();
}

MelmanMpcWrapper::~MelmanMpcWrapper()
{
    if(ocp)
        delete ocp;
    if(h)
        delete h;
    if(ref_traj)
        delete ref_traj;
    if(ref_traj_grid)
        delete ref_traj_grid;
    if(alg)
        delete alg;
    if(controller)
        delete controller;
}

void MelmanMpcWrapper::setupODE()
{
    f << dot(theta) == u2;
    f << dot(xs)    == c * u1 * cos(theta);
    f << dot(ys)    == c * u1 * sin(theta);
    f << dot(xc)    == c * u1 * cos(theta) + d * sin(theta) * u2;
    f << dot(yc)    == c * u1 * sin(theta) + d * cos(theta) * u2;
}

void MelmanMpcWrapper::updatePath(Legs const & legs, Point state)
{
    ROS_INFO("[mpc] 0");

    if(ocp)
    {
        delete ocp;
        ocp = 0;
    }
    if(h)
    {
        delete h;
        h = 0;
    }
    if(ref_traj)
    {
        delete ref_traj;
        ref_traj = 0;
    }
    if(ref_traj_grid)
    {
        delete ref_traj_grid;
        ref_traj_grid = 0;
    }
    if(alg)
    {
        delete alg;
        alg = 0;
    }
    if(controller)
    {
        delete controller;
        controller = 0;
    }

    ROS_INFO("[mpc] 1");

    double tend = std::accumulate(legs.begin(), legs.end(), 0.0, [](double const & acc, Leg const & l)
                                  { return acc + l.length / (l.speed * 0.4); });
    int    samplings = tend / 0.1;

    ref_traj_grid = new VariablesGrid(4, 0, tend, legs.size());
    ref_traj_grid->setZero();

    ref_traj_grid->setVector(0, DVector({legs.front().p1.x, legs.front().p2.y, 0, 0}));
    for(unsigned i = 0; i < legs.size(); i++)
        ref_traj_grid->setVector(0, DVector({legs[i].p2.x, legs[i].p2.y, 0, 0}));

    ROS_INFO("[mpc] 2");
    ocp      = new OCP(0.0, tend, samplings);
    h        = new Function;
    ref_traj = new StaticReferenceTrajectory(*ref_traj_grid);
    ROS_INFO("[mpc] 3");

    *h << xc;
    *h << yc;
    *h << u1;
    *h << u2;

    Q(0,0) = 1.0;
    Q(1,1) = 1.0;
    Q(2,2) = 0.05;
    Q(3,3) = 0.05;

    r.setAll(0.0) ;

    ocp->minimizeLSQ(Q, *h, r);
    ocp->subjectTo(f);
    ocp->subjectTo(AT_START, theta == 0.0);
    ocp->subjectTo(AT_START, xs == 0.0);
    ocp->subjectTo(AT_START, ys == 0.0);
    ocp->subjectTo(AT_START, xc == -d);
    ocp->subjectTo(AT_START, yc == 0.0);
    ocp->subjectTo(-mp.max_controller_speed_        <= u1 <= mp.max_controller_speed_);
    ocp->subjectTo(-mp.max_controller_angular_rate_ <= u2 <= mp.max_controller_angular_rate_);

    alg = new RealTimeAlgorithm(*ocp, 0.05);
    alg->set(MAX_NUM_ITERATIONS, 1);
    alg->set(PLOT_RESOLUTION, HIGH);
    alg->set(PRINTLEVEL, LOW);

    controller = new Controller(*alg, *ref_traj);
    DVector y( 5 ) ;

    y(0) = state.orientation;
    y(1) = state.x + std::cos(state.orientation) * d;
    y(2) = state.y + std::sin(state.orientation) * d;
    y(3) = state.x;
    y(4) = state.y;

    controller->init(0.0, y);
}

geometry_msgs::Twist MelmanMpcWrapper::feedbackStep(Point state, double t)
{
    DVector y( 5 ) ;
    y.setZero();
    y(0) = state.orientation;
    y(1) = state.x + std::cos(state.orientation) * d;
    y(2) = state.y + std::sin(state.orientation) * d;
    y(3) = state.x;
    y(4) = state.y;

    controller->step(t, y);

    DVector u;
    controller->getU(u);
    ROS_INFO("[mpc fb] ang rate = %f, lin speed = %f", u[1], u[0]);

    geometry_msgs::Twist twist;
    twist.angular.z = u[1];
    twist.linear.x  = u[0];
    return twist;
}

