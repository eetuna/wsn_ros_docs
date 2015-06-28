//experiment with Eigen library...
// wsn
//  May 1, 2013
// note inclusions in CMakeLists.txt and manifest.xml
// shows how to define and initialize matrices and vectors
// how to perform matrix-vector math: M*x, dot, cross, norm
// how to solve b = A*x (2 methods--Householder more general?) 
// demonstrated for 6x6 matrix eqns

// probably should use:  
// using namespace Eigen;
// using namespace std;


#include <ros/ros.h>
#include <stdio.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> // needed this for matrix eqn solver partialPivLu



int main(int argc, char** argv) {

    ros::init(argc, argv, "eigen_test");
    ros::NodeHandle nh_jntPub; // node handle for joint command publisher

 //   ros::Publisher pub_joint_commands; //
//    pub_joint_commands = nh_jntPub.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1, true); 
    
    ROS_INFO("test eigen program");
    
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    A << 1,2,3, 4,5,6, 7,8,10;
    
    A(1,2)=0; // how to access one element of matrix; start from 0; no warning out of range...
    
 
    b << 3,3,4;
    std::cout <<"b = "<<b <<std::endl;   
    
    // column operaton: replace first column of A with vector b:
    A.col(0)= b;  // could copy columns of matrices w/ A.col(0) = B.col(0);
    
    std::cout <<"A = "<<A <<std::endl;

    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Zero(6, 6); //6x6 matrix full of zeros
    Eigen::MatrixXd mat2 = Eigen::MatrixXd::Identity(6, 6); //6x6 identity matrix  

    std::cout<<mat1<<std::endl;
    std::cout<<mat2<<std::endl;

    Eigen::Vector3f xtest = A.colPivHouseholderQr().solve(b);
    std::cout<<"soln xtest = "<<xtest<<std::endl;
    
    Eigen::Vector3f x = A.partialPivLu().solve(b); //dec.solve(b); //A.colPivHouseholderQr().solve(b);
    std::cout<<"soln x = "<<x<<std::endl;
    
    Eigen::Vector3f btest = A*x;
    std::cout<<"test soln: A*x = " <<btest<<std::endl;
    
    //extend to 6x6 test: v = M*z, find z using 2 methods
    // use double-precision matrices/vectors
    Eigen::MatrixXd M = Eigen::MatrixXd::Random(6,6);

    std::cout<<"test 6x6: M = "<<M<<std::endl;
    Eigen::VectorXd v(6);   
    v << 1,2,3,4,5,6;
    std::cout<<"v = "<<v<<std::endl;
    Eigen::VectorXd z(6); 
    Eigen::VectorXd ztest(6);   
    ztest = M.colPivHouseholderQr().solve(v);
    std::cout<<"soln ztest = "<<ztest<<std::endl;
    z = M.partialPivLu().solve(v);   
    std::cout<<"soln 6x6: z = "<<z<<std::endl;
    Eigen::VectorXd vtest(6);
    vtest = M*z;
    std::cout<<"test soln: M*z = "<<vtest<<std::endl;

    // .norm() operator...
    double relative_error = (M*z - v).norm() / v.norm(); // norm() is L2 norm
    std::cout << "The relative error is:\n" << relative_error << std::endl;

    
    std::cout<<"dot prod, v, z: "<< v.dot(z)<<std::endl;
    std::cout<<"cross prod, b-cross-x: " << b.cross(x)<<std::endl;
    
    
    
    return 0;
}

