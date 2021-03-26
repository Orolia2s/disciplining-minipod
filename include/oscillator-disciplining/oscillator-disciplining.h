/**
 * @mainpage liboscillator-disciplining
 *
 * C library for disciplining an oscillator using miniCod algorithm.
 *
 * @see oscillator-disciplining.h for API documentation.
 */
/**
 * @file oscillator-disciplining.h
 * @brief liboscillator-disciplining's API header file.
 *
 * liboscillator-disciplining is a small library responsible of abstracting
 * disciplining algorithms used for an oscillator for which we want to control
 * the frequency.
 *
 */

#ifndef INCLUDE_OSCILLATOR_DISCIPLINING_OSCILLATOR_DISCIPLINING_H_
#define INCLUDE_OSCILLATOR_DISCIPLINING_OSCILLATOR_DISCIPLINING_H_
#include <time.h>
#include <stdbool.h>
#include <inttypes.h>

/**
 * @def OD_ERR_MSG_LEN
 * @brief Required size an error message buffer must have.
 */
#define OD_ERR_MSG_LEN 0x400

/**
 * @struct od_input
 * @brief Structure containing all the input parameters for the disciplining
 * algorithm.
 */
struct od_input {
	/** phase error measured between the oscillator and the GNSS */
	struct timespec phase_error;
	/** is GNSS available (and hence, is the phase error meaningful) */
	bool valid;
	/** is mRO locked */
	bool lock;
	/** temperature, only used for logging. */
	uint16_t temperature;
	/** Quantization error for the next PPS (unit is ps) */
	int32_t qErr;
	/** Fine adjustement setpoint */
	uint32_t fine_setpoint;
	/** Coarse adjustement setpoint */
	uint32_t coarse_setpoint;
};

/**
 * @enum output_action
 * @brief Enumeration of the possible action that must be done to control
 * the oscillator
 */
enum output_action {
	NO_OP,
	PHASE_JUMP,
	ADJUST_FINE,
	ADJUST_COARSE,
	CALIBRATE,
	NUM_ACTIONS,
};

/**
 * @struct od_output
 * @brief Structure which will be fed with the algorithm's outputs after
 * returning from od_process.
 */

struct od_output {
	/** frequence adjustment value, ranges depend wether concerning
	 * fine or coarse alignement:
	 * coarse: [0, 4194303]
	 * fine: [1600, 3200]
	 */
	uint32_t setpoint;
	/** Indicate action that should be done */
	enum output_action action;
	/** value of the phase jump */
	int32_t value_phase_ctrl;
};

/**
 * @struct od
 * @brief Opaque library context.
 */
struct od;

/**
 * @struct calibration_parameters
 * @brief structure used to pass parameters for mRO50's calibration
 */
struct calibration_parameters {
	/** control points used for the calibration */
	uint16_t *ctrl_points;
	/** Number of control points */
	int length;
	/** For each control points, nb_calibration measures 
	 * of the phase error must be done
	 */
	int nb_calibration;
	/** Interval between 2 measures */
	int settling_time;
};

/**
 * @struct calibration_results
 * @brief structure containing the results of a calibration done
 * from calibration_parameters structures.
 */
struct calibration_results {
	/** all results are stored in timespec double array of pointer
	 * calib_results can be represented as
	 * a double array of size length * nb_calibration
	*/
	struct timespec * measures;
	/** Number of control points.
	 * Should be equal to calibration_parameters.length
	 */
	int length;
	/** For each control points, number of phase error measures done.
	 * Should be equal to calibration_parameters.nb_calibration
	 */
	int nb_calibration;
};


/**
 * @brief Creates a liboscillator-disciplining context.
 * @param path Configuration file holding the configuration values for the
 * library.
 * @param err_msg Char buffer of size OD_ERR_MSG_LEN which will contain
 * additional human readable information on failure.
 * @return Context newly created, which must be destroyed by calling
 * od_destroy().
 */
struct od *od_new_from_config(const char *path, char err_msg[OD_ERR_MSG_LEN]);

/**
 * @brief Processes input data using the disciplining algorithm.
 * @param od Library context.
 * @param input Input parameters fed into the algorithm.
 * @param output Output data produced by the algorithm step ran.
 * @return 0 on success, errno-compatible negative value or error.
 */
int od_process(struct od *od, const struct od_input *input,
		struct od_output *output);


/**
 * @brief Outputs parameters needed for the mRO50 calibration.
 * @param od Library context.
 * @return struct calibration_parameters.
 */
struct calibration_parameters * od_get_calibration_parameters(struct od *od);

/**
 * @brief Process calibration results to compute phase time and ctrl coeffs.
 * @param od Library context.
 * @param calib_params calibration parameters obtained from od_get_calibration_paramters.
 * @param calib_results calibration measures done by the daemon.
 * @return 0 on success, errno-compatible negative value or error.
 */

void od_calibrate(struct od *od, struct calibration_parameters *calib_params, struct calibration_results *calib_results);

/**
 * @brief Destroys a context previously created by a call to od_new().
 * @param od Context to destroy, put to NULL in output, can be NULL or point to
 * NULL.
 */
void od_destroy(struct od **od);

#endif /* INCLUDE_OSCILLATOR_DISCIPLINING_OSCILLATOR_DISCIPLINING_H_ */
