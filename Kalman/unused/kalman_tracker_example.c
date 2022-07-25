/*!
* \brief This is demo code for the Kalman Filter factory includes.
*        It sets up some fake Kalman filters with measurements.
*/

/************************************************************************/
/* Create the first filter                                              */
/************************************************************************/

// create the filter structure

//#include "kalman_tracker_example.h"
//
//#define KALMAN_NAME tracker
//#define KALMAN_NUM_STATES 4
//#define KALMAN_NUM_INPUTS 0
//#include "kalman_factory_filter.h"
//
//// create the measurement structure
//#define KALMAN_MEASUREMENT_NAME position
//#define KALMAN_NUM_MEASUREMENTS 2
//#include "kalman_factory_measurement.h"
//
//#include <stdio.h>
//
///************************************************************************/
///* Initialization functions                                             */
///************************************************************************/
//
///*!
//* An example for the Kalman Filter factories
//*/
////static void test_kalman_1()
////{
////
////    kalman_tracker_model_init();
////    kalman_tracker_demo();
////
////
////    /*kalman_filter_tracker_init();*/
////    //kalman_filter_tracker_measurement_gyroscope_init();
////    //kalman_filter_tracker_measurement_accelerometer_init();
////
////    //kalman_filter_tracker.x.data[0] = 1;
////    //kalman_filter_tracker_measurement_gyroscope.z.data[0] = 1;
////    //kalman_filter_tracker_measurement_accelerometer.z.data[0] = 1;
////
////}
//
///*!
//* \brief Runs the gravity Kalman filter.
//*/
//void kalman_tracker_demo()
//{
//    // initialize the filter
//    //kalman_filter_tracker_init();
//
//    // fetch structures
//    kalman_t* kf = &kalman_filter_tracker;
//    kalman_measurement_t* kfm = &kalman_filter_tracker_measurement_position;
//
//    matrix_t* x = kalman_get_state_vector(kf);
//    matrix_t* z = kalman_get_measurement_vector(kfm);
//
//    // filter!
//# define MEAS_COUNT 10
//    for (int i = 0; i < MEAS_COUNT; ++i)
//    {
//        // prediction.
//        kalman_predict(kf);
//
//        // measure ...
//        matrix_data_t measured_position[2] = { 1, -1 };
//        matrix_set(z, 0, 0, measured_position[0]);
//        matrix_set(z, 1, 0, measured_position[1]);
//
//        // update
//        kalman_correct(kf, kfm);
//
//        printf("State x: %f, y: %f\r\n", x->data[0], x->data[2]);
//
//    }
//
//    //// fetch estimated g
//    //matrix_data_t g_estimated = x->data[2];
//    //assert(g_estimated > 9 && g_estimated < 10);
//}
//
//void kalman_tracker_model_init()
//{
//    /************************************************************************/
//    /* initialize the filter structures                                     */
//    /************************************************************************/
//    kalman_t* kf = kalman_filter_tracker_init();
//    kalman_measurement_t* kfm = kalman_filter_tracker_measurement_position_init();
//
//    /************************************************************************/
//    /* Time constant and variance setup                                     */
//    /************************************************************************/
//    const matrix_data_t T = 1.0f;
//    matrix_data_t state_var = 20.0f; // Q - should state variance be the same for x and x' ?
//    matrix_data_t measurement_var = 40.0f; //
//
//    /************************************************************************/
//    /* set initial state                                                    */
//    /************************************************************************/
//    matrix_t* x = kalman_get_state_vector(kf);
//    x->data[0] = 0; // x
//    x->data[1] = 0; // x'
//    x->data[2] = 0; // y
//    x->data[3] = 0; // y'
//
//    /************************************************************************/
//    /* set state transition                                                 */
//    /************************************************************************/
//    matrix_t* A = kalman_get_state_transition(kf);
//
//    //# initalize Kalman matrices
//    //    self.A = np.matrix([  [1, self.dt,    0, 0 ],
//    //                          [0, 1,          0, 0],
//    //                          [0, 0,          1, self.dt],
//    //                          [0, 0,          0, 1]] )
//
//    // transition in X direction
//    matrix_set(A, 0, 0, 1);   // 1
//    matrix_set(A, 0, 1, T);   // T
//    matrix_set(A, 1, 1, 1);   // 1
//
//    // transition in Y direction
//    matrix_set(A, 2, 2, 1);   // 1
//    matrix_set(A, 2, 3, T);   // T
//    matrix_set(A, 3, 3, 1);   // 1
//
//    /************************************************************************/
//    /* set system covariance                                                */
//    /************************************************************************/
//
//    //self.P = np.matrix(self.stateVariance * np.identity(self.A.shape[0])) # 4x4
//
//    matrix_t* P = kalman_get_system_covariance(kf);
//
//    matrix_set(P, 0, 0, state_var);
//    matrix_set(P, 1, 1, state_var);
//    matrix_set(P, 2, 2, state_var);
//    matrix_set(P, 3, 3, state_var);
//
//    /************************************************************************/
//    /* set process covariance covariance                                                 */
//    /************************************************************************/
//
//    //self.G = np.matrix([[1 / 2 * self.dt * *2, 0 ],
//    //    [self.dt, 0],
//    //    [0, 1 / 2 * self.dt * *2],
//    //    [0, self.dt]] )
//    //self.Q = self.G * self.G.T * self.stateVariance
//
//    // proce covariance - not initialized... Throws exception
//    matrix_t* Q = kalman_get_input_covariance(kf);
//
//    matrix_set(Q, 0, 0, (matrix_data_t)0.25*T*T*T*T*state_var);
//    matrix_set_symmetric(Q, 0, 1, (matrix_data_t)0.5*T*T*state_var);
//    matrix_set(Q, 1, 1, T*state_var);
//
//    matrix_set(Q, 2, 2, (matrix_data_t)0.25*T*T*T*T*state_var);
//    matrix_set_symmetric(Q, 2, 3, (matrix_data_t)0.5*T*T*state_var);
//    matrix_set(Q, 3, 3, (matrix_data_t)T*state_var);
//
//    /************************************************************************/
//    /* set measurement transformation                                       */
//    /************************************************************************/
//    //self.H = np.matrix([  [1, 0, 0, 0],
//    //                      [0, 0, 1, 0]] )
//    matrix_t* H = kalman_get_measurement_transformation(kfm);
//
//    matrix_set(H, 0, 0, 1); // x
//    matrix_set(H, 1, 2, 1); // y
//
//    /************************************************************************/
//    /* set process noise                                                    */
//    /************************************************************************/
//    // self.R = np.matrix(self.measurementVariance * np.identity(self.H.shape[0])) # 2x2
//    matrix_t* R = kalman_get_process_noise(kfm);
//
//    matrix_set(R, 0, 0, measurement_var);
//    matrix_set(R, 1, 1, measurement_var);
//}
//
//// clean up
//#include "kalman_factory_cleanup.h"