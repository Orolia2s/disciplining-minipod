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
 * @brief Creates a liboscillator-disciplining context.
 * @param clockid Type of clock to use for timestamping the input data.
 * @return Context newly created, which must be destroyed by calling
 * od_destroy().
 * @deprecated replaced by od_new_from_config
 */
struct od *od_new(clockid_t clockid) __attribute__((deprecated));

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
 * @brief Returns the minimum admissible value for the DAC setpoint.
 * @param od Library context
 * @return Minimum DAC setpoint value, which comes either from the value hard-
 * coded into the library, or from the ctrl.DACmin configuration entry, if
 * specified.
 */
uint32_t od_get_dac_min(const struct od *od);

/**
 * @brief Returns the maximum admissible value for the DAC setpoint.
 * @param od Library context
 * @return Maximum DAC setpoint value, which comes either from the value hard-
 * coded into the library, or from the ctrl.DACmax configuration entry, if
 * specified.
 */
uint32_t od_get_dac_max(const struct od *od);

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
 * @brief Destroys a context previously created by a call to od_new().
 * @param od Context to destroy, put to NULL in output, can be NULL or point to
 * NULL.
 */
void od_destroy(struct od **od);

#endif /* INCLUDE_OSCILLATOR_DISCIPLINING_OSCILLATOR_DISCIPLINING_H_ */
