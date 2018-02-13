#include <models.hpp>

Chain SnakeRobot(){
    Chain SnakeRobot;
    //it's a alpha d theta
//    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
//                                          Frame::DH_Craig1989(0.166, 0.0, 0.0, -M_PI_2))));
//    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
//                                          Frame::DH_Craig1989(0.0, -M_PI_2, 0.1167, 0.0))));
//    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
//                                          Frame::DH_Craig1989(0.0, M_PI_2, 0.0, M_PI_2))));
//    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
//                                          Frame::DH_Craig1989(0.0836, 0.0, 0.0, -M_PI_2))));
//    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
//                                          Frame::DH_Craig1989(0.0, -M_PI_2, 0.1167, 0.0))));
//    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
//                                          Frame::DH_Craig1989(0.0, M_PI_2, 0.0, M_PI_2))));
//    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::None),
//                                          Frame::DH_Craig1989(0.166, 0.0, 0.0, 0.0))));


    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::None),
                                          Frame::DH_Craig1989(0.166, 0.0, 0.0, -M_PI_2))));
    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
                                          Frame::DH_Craig1989(0.0, -M_PI_2, 0.1167, 0))));
    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
                                          Frame::DH_Craig1989(0.0, M_PI_2, 0.0, M_PI_2))));
    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
                                          Frame::DH_Craig1989(0.0836, 0.0, 0.0, -M_PI_2))));
    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
                                          Frame::DH_Craig1989(0.0, -M_PI_2, 0.1167, 0))));
    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
                                          Frame::DH_Craig1989(0.0, M_PI_2, 0.0, M_PI_2))));
    SnakeRobot.addSegment(Segment(Segment(Joint(Joint::RotZ),
                                          Frame::DH_Craig1989(0.166, 0.0, 0.0, 0.0))));


    return SnakeRobot;

}
