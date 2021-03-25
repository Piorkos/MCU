/**
  ******************************************************************************
  * @file    predict_house_price.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Thu Mar 25 12:32:24 2021
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


#include "predict_house_price.h"

#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



/*
#define AI_TOOLS_VERSION_MAJOR 6
#define AI_TOOLS_VERSION_MINOR 0
#define AI_TOOLS_VERSION_MICRO 0
#define AI_TOOLS_VERSION_EXTRA "RC6"

*/

/*
#define AI_TOOLS_API_VERSION_MAJOR 1
#define AI_TOOLS_API_VERSION_MINOR 4
#define AI_TOOLS_API_VERSION_MICRO 0
*/

#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_predict_house_price
 
#undef AI_PREDICT_HOUSE_PRICE_MODEL_SIGNATURE
#define AI_PREDICT_HOUSE_PRICE_MODEL_SIGNATURE     "642e370b38361999b23bd673f23a6faa"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Thu Mar 25 12:32:24 2021"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_PREDICT_HOUSE_PRICE_N_BATCHES
#define AI_PREDICT_HOUSE_PRICE_N_BATCHES         (1)

/**  Forward network declaration section  *************************************/
AI_STATIC ai_network AI_NET_OBJ_INSTANCE;


/**  Forward network array declarations  **************************************/
AI_STATIC ai_array dense_73_input_output_array;   /* Array #0 */
AI_STATIC ai_array dense_0_output_array;   /* Array #1 */
AI_STATIC ai_array nl_0_nl_output_array;   /* Array #2 */
AI_STATIC ai_array dense_1_output_array;   /* Array #3 */
AI_STATIC ai_array nl_1_nl_output_array;   /* Array #4 */
AI_STATIC ai_array dense_2_output_array;   /* Array #5 */
AI_STATIC ai_array nl_2_nl_output_array;   /* Array #6 */
AI_STATIC ai_array dense_3_output_array;   /* Array #7 */
AI_STATIC ai_array nl_3_nl_output_array;   /* Array #8 */
AI_STATIC ai_array dense_4_output_array;   /* Array #9 */
AI_STATIC ai_array nl_4_nl_output_array;   /* Array #10 */
AI_STATIC ai_array dense_5_output_array;   /* Array #11 */
AI_STATIC ai_array nl_5_nl_output_array;   /* Array #12 */
AI_STATIC ai_array dense_6_output_array;   /* Array #13 */
AI_STATIC ai_array nl_6_nl_output_array;   /* Array #14 */
AI_STATIC ai_array dense_7_output_array;   /* Array #15 */
AI_STATIC ai_array nl_7_nl_output_array;   /* Array #16 */
AI_STATIC ai_array dense_8_output_array;   /* Array #17 */
AI_STATIC ai_array nl_8_nl_output_array;   /* Array #18 */
AI_STATIC ai_array dense_9_output_array;   /* Array #19 */
AI_STATIC ai_array nl_9_nl_output_array;   /* Array #20 */
AI_STATIC ai_array dense_10_output_array;   /* Array #21 */
AI_STATIC ai_array nl_10_nl_output_array;   /* Array #22 */
AI_STATIC ai_array dense_0_weights_array;   /* Array #23 */
AI_STATIC ai_array dense_0_bias_array;   /* Array #24 */
AI_STATIC ai_array dense_1_weights_array;   /* Array #25 */
AI_STATIC ai_array dense_1_bias_array;   /* Array #26 */
AI_STATIC ai_array dense_2_weights_array;   /* Array #27 */
AI_STATIC ai_array dense_2_bias_array;   /* Array #28 */
AI_STATIC ai_array dense_3_weights_array;   /* Array #29 */
AI_STATIC ai_array dense_3_bias_array;   /* Array #30 */
AI_STATIC ai_array dense_4_weights_array;   /* Array #31 */
AI_STATIC ai_array dense_4_bias_array;   /* Array #32 */
AI_STATIC ai_array dense_5_weights_array;   /* Array #33 */
AI_STATIC ai_array dense_5_bias_array;   /* Array #34 */
AI_STATIC ai_array dense_6_weights_array;   /* Array #35 */
AI_STATIC ai_array dense_6_bias_array;   /* Array #36 */
AI_STATIC ai_array dense_7_weights_array;   /* Array #37 */
AI_STATIC ai_array dense_7_bias_array;   /* Array #38 */
AI_STATIC ai_array dense_8_weights_array;   /* Array #39 */
AI_STATIC ai_array dense_8_bias_array;   /* Array #40 */
AI_STATIC ai_array dense_9_weights_array;   /* Array #41 */
AI_STATIC ai_array dense_9_bias_array;   /* Array #42 */
AI_STATIC ai_array dense_10_weights_array;   /* Array #43 */
AI_STATIC ai_array dense_10_bias_array;   /* Array #44 */


/**  Forward network tensor declarations  *************************************/
AI_STATIC ai_tensor dense_73_input_output;   /* Tensor #0 */
AI_STATIC ai_tensor dense_0_output;   /* Tensor #1 */
AI_STATIC ai_tensor nl_0_nl_output;   /* Tensor #2 */
AI_STATIC ai_tensor dense_1_output;   /* Tensor #3 */
AI_STATIC ai_tensor nl_1_nl_output;   /* Tensor #4 */
AI_STATIC ai_tensor dense_2_output;   /* Tensor #5 */
AI_STATIC ai_tensor nl_2_nl_output;   /* Tensor #6 */
AI_STATIC ai_tensor dense_3_output;   /* Tensor #7 */
AI_STATIC ai_tensor nl_3_nl_output;   /* Tensor #8 */
AI_STATIC ai_tensor dense_4_output;   /* Tensor #9 */
AI_STATIC ai_tensor nl_4_nl_output;   /* Tensor #10 */
AI_STATIC ai_tensor dense_5_output;   /* Tensor #11 */
AI_STATIC ai_tensor nl_5_nl_output;   /* Tensor #12 */
AI_STATIC ai_tensor dense_6_output;   /* Tensor #13 */
AI_STATIC ai_tensor nl_6_nl_output;   /* Tensor #14 */
AI_STATIC ai_tensor dense_7_output;   /* Tensor #15 */
AI_STATIC ai_tensor nl_7_nl_output;   /* Tensor #16 */
AI_STATIC ai_tensor dense_8_output;   /* Tensor #17 */
AI_STATIC ai_tensor nl_8_nl_output;   /* Tensor #18 */
AI_STATIC ai_tensor dense_9_output;   /* Tensor #19 */
AI_STATIC ai_tensor nl_9_nl_output;   /* Tensor #20 */
AI_STATIC ai_tensor dense_10_output;   /* Tensor #21 */
AI_STATIC ai_tensor nl_10_nl_output;   /* Tensor #22 */
AI_STATIC ai_tensor dense_0_weights;   /* Tensor #23 */
AI_STATIC ai_tensor dense_0_bias;   /* Tensor #24 */
AI_STATIC ai_tensor dense_1_weights;   /* Tensor #25 */
AI_STATIC ai_tensor dense_1_bias;   /* Tensor #26 */
AI_STATIC ai_tensor dense_2_weights;   /* Tensor #27 */
AI_STATIC ai_tensor dense_2_bias;   /* Tensor #28 */
AI_STATIC ai_tensor dense_3_weights;   /* Tensor #29 */
AI_STATIC ai_tensor dense_3_bias;   /* Tensor #30 */
AI_STATIC ai_tensor dense_4_weights;   /* Tensor #31 */
AI_STATIC ai_tensor dense_4_bias;   /* Tensor #32 */
AI_STATIC ai_tensor dense_5_weights;   /* Tensor #33 */
AI_STATIC ai_tensor dense_5_bias;   /* Tensor #34 */
AI_STATIC ai_tensor dense_6_weights;   /* Tensor #35 */
AI_STATIC ai_tensor dense_6_bias;   /* Tensor #36 */
AI_STATIC ai_tensor dense_7_weights;   /* Tensor #37 */
AI_STATIC ai_tensor dense_7_bias;   /* Tensor #38 */
AI_STATIC ai_tensor dense_8_weights;   /* Tensor #39 */
AI_STATIC ai_tensor dense_8_bias;   /* Tensor #40 */
AI_STATIC ai_tensor dense_9_weights;   /* Tensor #41 */
AI_STATIC ai_tensor dense_9_bias;   /* Tensor #42 */
AI_STATIC ai_tensor dense_10_weights;   /* Tensor #43 */
AI_STATIC ai_tensor dense_10_bias;   /* Tensor #44 */


/**  Forward network tensor chain declarations  *******************************/
AI_STATIC_CONST ai_tensor_chain dense_0_chain;   /* Chain #0 */
AI_STATIC_CONST ai_tensor_chain nl_0_nl_chain;   /* Chain #1 */
AI_STATIC_CONST ai_tensor_chain dense_1_chain;   /* Chain #2 */
AI_STATIC_CONST ai_tensor_chain nl_1_nl_chain;   /* Chain #3 */
AI_STATIC_CONST ai_tensor_chain dense_2_chain;   /* Chain #4 */
AI_STATIC_CONST ai_tensor_chain nl_2_nl_chain;   /* Chain #5 */
AI_STATIC_CONST ai_tensor_chain dense_3_chain;   /* Chain #6 */
AI_STATIC_CONST ai_tensor_chain nl_3_nl_chain;   /* Chain #7 */
AI_STATIC_CONST ai_tensor_chain dense_4_chain;   /* Chain #8 */
AI_STATIC_CONST ai_tensor_chain nl_4_nl_chain;   /* Chain #9 */
AI_STATIC_CONST ai_tensor_chain dense_5_chain;   /* Chain #10 */
AI_STATIC_CONST ai_tensor_chain nl_5_nl_chain;   /* Chain #11 */
AI_STATIC_CONST ai_tensor_chain dense_6_chain;   /* Chain #12 */
AI_STATIC_CONST ai_tensor_chain nl_6_nl_chain;   /* Chain #13 */
AI_STATIC_CONST ai_tensor_chain dense_7_chain;   /* Chain #14 */
AI_STATIC_CONST ai_tensor_chain nl_7_nl_chain;   /* Chain #15 */
AI_STATIC_CONST ai_tensor_chain dense_8_chain;   /* Chain #16 */
AI_STATIC_CONST ai_tensor_chain nl_8_nl_chain;   /* Chain #17 */
AI_STATIC_CONST ai_tensor_chain dense_9_chain;   /* Chain #18 */
AI_STATIC_CONST ai_tensor_chain nl_9_nl_chain;   /* Chain #19 */
AI_STATIC_CONST ai_tensor_chain dense_10_chain;   /* Chain #20 */
AI_STATIC_CONST ai_tensor_chain nl_10_nl_chain;   /* Chain #21 */


/**  Forward network layer declarations  **************************************/
AI_STATIC ai_layer_dense dense_0_layer; /* Layer #0 */
AI_STATIC ai_layer_nl nl_0_nl_layer; /* Layer #1 */
AI_STATIC ai_layer_dense dense_1_layer; /* Layer #2 */
AI_STATIC ai_layer_nl nl_1_nl_layer; /* Layer #3 */
AI_STATIC ai_layer_dense dense_2_layer; /* Layer #4 */
AI_STATIC ai_layer_nl nl_2_nl_layer; /* Layer #5 */
AI_STATIC ai_layer_dense dense_3_layer; /* Layer #6 */
AI_STATIC ai_layer_nl nl_3_nl_layer; /* Layer #7 */
AI_STATIC ai_layer_dense dense_4_layer; /* Layer #8 */
AI_STATIC ai_layer_nl nl_4_nl_layer; /* Layer #9 */
AI_STATIC ai_layer_dense dense_5_layer; /* Layer #10 */
AI_STATIC ai_layer_nl nl_5_nl_layer; /* Layer #11 */
AI_STATIC ai_layer_dense dense_6_layer; /* Layer #12 */
AI_STATIC ai_layer_nl nl_6_nl_layer; /* Layer #13 */
AI_STATIC ai_layer_dense dense_7_layer; /* Layer #14 */
AI_STATIC ai_layer_nl nl_7_nl_layer; /* Layer #15 */
AI_STATIC ai_layer_dense dense_8_layer; /* Layer #16 */
AI_STATIC ai_layer_nl nl_8_nl_layer; /* Layer #17 */
AI_STATIC ai_layer_dense dense_9_layer; /* Layer #18 */
AI_STATIC ai_layer_nl nl_9_nl_layer; /* Layer #19 */
AI_STATIC ai_layer_dense dense_10_layer; /* Layer #20 */
AI_STATIC ai_layer_nl nl_10_nl_layer; /* Layer #21 */




/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  dense_73_input_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 19, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  dense_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  nl_0_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  nl_1_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  dense_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  nl_2_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  nl_3_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  nl_4_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  nl_5_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  dense_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  nl_6_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  dense_7_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  nl_7_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  dense_8_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  nl_8_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  dense_9_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  nl_9_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  dense_10_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  nl_10_nl_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 1, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  dense_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 361, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  dense_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 361, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  dense_2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 722, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  dense_2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1444, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1444, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 38, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 722, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  dense_6_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 361, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  dense_6_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#37 */
AI_ARRAY_OBJ_DECLARE(
  dense_7_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 361, AI_STATIC)

/* Array#38 */
AI_ARRAY_OBJ_DECLARE(
  dense_7_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#39 */
AI_ARRAY_OBJ_DECLARE(
  dense_8_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 361, AI_STATIC)

/* Array#40 */
AI_ARRAY_OBJ_DECLARE(
  dense_8_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 19, AI_STATIC)

/* Array#41 */
AI_ARRAY_OBJ_DECLARE(
  dense_9_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 152, AI_STATIC)

/* Array#42 */
AI_ARRAY_OBJ_DECLARE(
  dense_9_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)

/* Array#43 */
AI_ARRAY_OBJ_DECLARE(
  dense_10_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)

/* Array#44 */
AI_ARRAY_OBJ_DECLARE(
  dense_10_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  dense_73_input_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_73_input_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  dense_0_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_0_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  nl_0_nl_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &nl_0_nl_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_1_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  nl_1_nl_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &nl_1_nl_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  dense_2_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &dense_2_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  nl_2_nl_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &nl_2_nl_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &dense_3_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  nl_3_nl_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &nl_3_nl_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &dense_4_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  nl_4_nl_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &nl_4_nl_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_output, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_5_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  nl_5_nl_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &nl_5_nl_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  dense_6_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_6_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  nl_6_nl_output, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &nl_6_nl_output_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  dense_7_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_7_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  nl_7_nl_output, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &nl_7_nl_output_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  dense_8_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_8_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  nl_8_nl_output, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &nl_8_nl_output_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  dense_9_output, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &dense_9_output_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  nl_9_nl_output, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &nl_9_nl_output_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  dense_10_output, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &dense_10_output_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  nl_10_nl_output, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &nl_10_nl_output_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  dense_0_weights, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 19, 19, 1, 1), AI_STRIDE_INIT(4, 4, 76, 1444, 1444),
  1, &dense_0_weights_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  dense_0_bias, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_0_bias_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_weights, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 19, 19, 1, 1), AI_STRIDE_INIT(4, 4, 76, 1444, 1444),
  1, &dense_1_weights_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_bias, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_1_bias_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  dense_2_weights, AI_STATIC,
  27, 0x0,
  AI_SHAPE_INIT(4, 19, 38, 1, 1), AI_STRIDE_INIT(4, 4, 76, 2888, 2888),
  1, &dense_2_weights_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  dense_2_bias, AI_STATIC,
  28, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &dense_2_bias_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_weights, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 38, 38, 1, 1), AI_STRIDE_INIT(4, 4, 152, 5776, 5776),
  1, &dense_3_weights_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_bias, AI_STATIC,
  30, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &dense_3_bias_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_weights, AI_STATIC,
  31, 0x0,
  AI_SHAPE_INIT(4, 38, 38, 1, 1), AI_STRIDE_INIT(4, 4, 152, 5776, 5776),
  1, &dense_4_weights_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_bias, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 1, 38, 1, 1), AI_STRIDE_INIT(4, 4, 4, 152, 152),
  1, &dense_4_bias_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_weights, AI_STATIC,
  33, 0x0,
  AI_SHAPE_INIT(4, 38, 19, 1, 1), AI_STRIDE_INIT(4, 4, 152, 2888, 2888),
  1, &dense_5_weights_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_bias, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_5_bias_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  dense_6_weights, AI_STATIC,
  35, 0x0,
  AI_SHAPE_INIT(4, 19, 19, 1, 1), AI_STRIDE_INIT(4, 4, 76, 1444, 1444),
  1, &dense_6_weights_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  dense_6_bias, AI_STATIC,
  36, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_6_bias_array, NULL)

/* Tensor #37 */
AI_TENSOR_OBJ_DECLARE(
  dense_7_weights, AI_STATIC,
  37, 0x0,
  AI_SHAPE_INIT(4, 19, 19, 1, 1), AI_STRIDE_INIT(4, 4, 76, 1444, 1444),
  1, &dense_7_weights_array, NULL)

/* Tensor #38 */
AI_TENSOR_OBJ_DECLARE(
  dense_7_bias, AI_STATIC,
  38, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_7_bias_array, NULL)

/* Tensor #39 */
AI_TENSOR_OBJ_DECLARE(
  dense_8_weights, AI_STATIC,
  39, 0x0,
  AI_SHAPE_INIT(4, 19, 19, 1, 1), AI_STRIDE_INIT(4, 4, 76, 1444, 1444),
  1, &dense_8_weights_array, NULL)

/* Tensor #40 */
AI_TENSOR_OBJ_DECLARE(
  dense_8_bias, AI_STATIC,
  40, 0x0,
  AI_SHAPE_INIT(4, 1, 19, 1, 1), AI_STRIDE_INIT(4, 4, 4, 76, 76),
  1, &dense_8_bias_array, NULL)

/* Tensor #41 */
AI_TENSOR_OBJ_DECLARE(
  dense_9_weights, AI_STATIC,
  41, 0x0,
  AI_SHAPE_INIT(4, 19, 8, 1, 1), AI_STRIDE_INIT(4, 4, 76, 608, 608),
  1, &dense_9_weights_array, NULL)

/* Tensor #42 */
AI_TENSOR_OBJ_DECLARE(
  dense_9_bias, AI_STATIC,
  42, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &dense_9_bias_array, NULL)

/* Tensor #43 */
AI_TENSOR_OBJ_DECLARE(
  dense_10_weights, AI_STATIC,
  43, 0x0,
  AI_SHAPE_INIT(4, 8, 1, 1, 1), AI_STRIDE_INIT(4, 4, 32, 32, 32),
  1, &dense_10_weights_array, NULL)

/* Tensor #44 */
AI_TENSOR_OBJ_DECLARE(
  dense_10_bias, AI_STATIC,
  44, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &dense_10_bias_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_73_input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_0_weights, &dense_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_0_layer, 0,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_0_chain,
  &AI_NET_OBJ_INSTANCE, &nl_0_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_0_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_0_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_0_nl_layer, 0,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_0_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_0_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_1_weights, &dense_1_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_layer, 1,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_1_chain,
  &AI_NET_OBJ_INSTANCE, &nl_1_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_1_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_1_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_1_nl_layer, 1,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_1_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_1_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_2_weights, &dense_2_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_2_layer, 2,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_2_chain,
  &AI_NET_OBJ_INSTANCE, &nl_2_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_2_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_2_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_2_nl_layer, 2,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_2_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_3_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_2_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_3_weights, &dense_3_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_3_layer, 3,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_3_chain,
  &AI_NET_OBJ_INSTANCE, &nl_3_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_3_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_3_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_3_nl_layer, 3,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_3_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_4_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_3_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_4_weights, &dense_4_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_4_layer, 4,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_4_chain,
  &AI_NET_OBJ_INSTANCE, &nl_4_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_4_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_4_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_4_nl_layer, 4,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_4_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_5_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_4_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_5_weights, &dense_5_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_5_layer, 5,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_5_chain,
  &AI_NET_OBJ_INSTANCE, &nl_5_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_5_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_5_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_5_nl_layer, 5,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_5_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_6_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_5_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_6_weights, &dense_6_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_6_layer, 6,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_6_chain,
  &AI_NET_OBJ_INSTANCE, &nl_6_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_6_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_6_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_6_nl_layer, 6,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_6_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_7_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_7_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_6_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_7_weights, &dense_7_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_7_layer, 7,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_7_chain,
  &AI_NET_OBJ_INSTANCE, &nl_7_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_7_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_7_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_7_nl_layer, 7,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_7_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_8_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_8_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_7_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_8_weights, &dense_8_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_8_layer, 8,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_8_chain,
  &AI_NET_OBJ_INSTANCE, &nl_8_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_8_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_8_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_8_nl_layer, 8,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_8_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_9_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_9_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_8_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_9_weights, &dense_9_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_9_layer, 9,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_9_chain,
  &AI_NET_OBJ_INSTANCE, &nl_9_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_9_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_9_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_9_nl_layer, 9,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_9_nl_chain,
  &AI_NET_OBJ_INSTANCE, &dense_10_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_10_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_9_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_10_weights, &dense_10_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_10_layer, 10,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_10_chain,
  &AI_NET_OBJ_INSTANCE, &nl_10_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_10_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_10_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_10_nl_layer, 10,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &nl_10_nl_chain,
  &AI_NET_OBJ_INSTANCE, &nl_10_nl_layer, AI_STATIC, 
  .nl_params = NULL, 
)


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 26136, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 304, 1,
                     NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_PREDICT_HOUSE_PRICE_IN_NUM, &dense_73_input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_PREDICT_HOUSE_PRICE_OUT_NUM, &nl_10_nl_output),
  &dense_0_layer, 0, NULL)



AI_DECLARE_STATIC
ai_bool predict_house_price_configure_activations(
  ai_network* net_ctx, const ai_buffer* activation_buffer)
{
  AI_ASSERT(net_ctx &&  activation_buffer && activation_buffer->data)

  ai_ptr activations = AI_PTR(AI_PTR_ALIGN(activation_buffer->data, 4));
  AI_ASSERT(activations)
  AI_UNUSED(net_ctx)

  {
    /* Updating activations (byte) offsets */
    dense_73_input_output_array.data = AI_PTR(NULL);
    dense_73_input_output_array.data_start = AI_PTR(NULL);
    dense_0_output_array.data = AI_PTR(activations + 76);
    dense_0_output_array.data_start = AI_PTR(activations + 76);
    nl_0_nl_output_array.data = AI_PTR(activations + 76);
    nl_0_nl_output_array.data_start = AI_PTR(activations + 76);
    dense_1_output_array.data = AI_PTR(activations + 152);
    dense_1_output_array.data_start = AI_PTR(activations + 152);
    nl_1_nl_output_array.data = AI_PTR(activations + 76);
    nl_1_nl_output_array.data_start = AI_PTR(activations + 76);
    dense_2_output_array.data = AI_PTR(activations + 152);
    dense_2_output_array.data_start = AI_PTR(activations + 152);
    nl_2_nl_output_array.data = AI_PTR(activations + 152);
    nl_2_nl_output_array.data_start = AI_PTR(activations + 152);
    dense_3_output_array.data = AI_PTR(activations + 0);
    dense_3_output_array.data_start = AI_PTR(activations + 0);
    nl_3_nl_output_array.data = AI_PTR(activations + 152);
    nl_3_nl_output_array.data_start = AI_PTR(activations + 152);
    dense_4_output_array.data = AI_PTR(activations + 0);
    dense_4_output_array.data_start = AI_PTR(activations + 0);
    nl_4_nl_output_array.data = AI_PTR(activations + 152);
    nl_4_nl_output_array.data_start = AI_PTR(activations + 152);
    dense_5_output_array.data = AI_PTR(activations + 0);
    dense_5_output_array.data_start = AI_PTR(activations + 0);
    nl_5_nl_output_array.data = AI_PTR(activations + 76);
    nl_5_nl_output_array.data_start = AI_PTR(activations + 76);
    dense_6_output_array.data = AI_PTR(activations + 0);
    dense_6_output_array.data_start = AI_PTR(activations + 0);
    nl_6_nl_output_array.data = AI_PTR(activations + 76);
    nl_6_nl_output_array.data_start = AI_PTR(activations + 76);
    dense_7_output_array.data = AI_PTR(activations + 0);
    dense_7_output_array.data_start = AI_PTR(activations + 0);
    nl_7_nl_output_array.data = AI_PTR(activations + 76);
    nl_7_nl_output_array.data_start = AI_PTR(activations + 76);
    dense_8_output_array.data = AI_PTR(activations + 0);
    dense_8_output_array.data_start = AI_PTR(activations + 0);
    nl_8_nl_output_array.data = AI_PTR(activations + 76);
    nl_8_nl_output_array.data_start = AI_PTR(activations + 76);
    dense_9_output_array.data = AI_PTR(activations + 0);
    dense_9_output_array.data_start = AI_PTR(activations + 0);
    nl_9_nl_output_array.data = AI_PTR(activations + 32);
    nl_9_nl_output_array.data_start = AI_PTR(activations + 32);
    dense_10_output_array.data = AI_PTR(activations + 0);
    dense_10_output_array.data_start = AI_PTR(activations + 0);
    nl_10_nl_output_array.data = AI_PTR(NULL);
    nl_10_nl_output_array.data_start = AI_PTR(NULL);
    
  }
  return true;
}



AI_DECLARE_STATIC
ai_bool predict_house_price_configure_weights(
  ai_network* net_ctx, const ai_buffer* weights_buffer)
{
  AI_ASSERT(net_ctx &&  weights_buffer && weights_buffer->data)

  ai_ptr weights = AI_PTR(weights_buffer->data);
  AI_ASSERT(weights)
  AI_UNUSED(net_ctx)

  {
    /* Updating weights (byte) offsets */
    
    dense_0_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_0_weights_array.data = AI_PTR(weights + 0);
    dense_0_weights_array.data_start = AI_PTR(weights + 0);
    dense_0_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_0_bias_array.data = AI_PTR(weights + 1444);
    dense_0_bias_array.data_start = AI_PTR(weights + 1444);
    dense_1_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_1_weights_array.data = AI_PTR(weights + 1520);
    dense_1_weights_array.data_start = AI_PTR(weights + 1520);
    dense_1_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_1_bias_array.data = AI_PTR(weights + 2964);
    dense_1_bias_array.data_start = AI_PTR(weights + 2964);
    dense_2_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_2_weights_array.data = AI_PTR(weights + 3040);
    dense_2_weights_array.data_start = AI_PTR(weights + 3040);
    dense_2_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_2_bias_array.data = AI_PTR(weights + 5928);
    dense_2_bias_array.data_start = AI_PTR(weights + 5928);
    dense_3_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_3_weights_array.data = AI_PTR(weights + 6080);
    dense_3_weights_array.data_start = AI_PTR(weights + 6080);
    dense_3_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_3_bias_array.data = AI_PTR(weights + 11856);
    dense_3_bias_array.data_start = AI_PTR(weights + 11856);
    dense_4_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_4_weights_array.data = AI_PTR(weights + 12008);
    dense_4_weights_array.data_start = AI_PTR(weights + 12008);
    dense_4_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_4_bias_array.data = AI_PTR(weights + 17784);
    dense_4_bias_array.data_start = AI_PTR(weights + 17784);
    dense_5_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_5_weights_array.data = AI_PTR(weights + 17936);
    dense_5_weights_array.data_start = AI_PTR(weights + 17936);
    dense_5_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_5_bias_array.data = AI_PTR(weights + 20824);
    dense_5_bias_array.data_start = AI_PTR(weights + 20824);
    dense_6_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_6_weights_array.data = AI_PTR(weights + 20900);
    dense_6_weights_array.data_start = AI_PTR(weights + 20900);
    dense_6_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_6_bias_array.data = AI_PTR(weights + 22344);
    dense_6_bias_array.data_start = AI_PTR(weights + 22344);
    dense_7_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_7_weights_array.data = AI_PTR(weights + 22420);
    dense_7_weights_array.data_start = AI_PTR(weights + 22420);
    dense_7_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_7_bias_array.data = AI_PTR(weights + 23864);
    dense_7_bias_array.data_start = AI_PTR(weights + 23864);
    dense_8_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_8_weights_array.data = AI_PTR(weights + 23940);
    dense_8_weights_array.data_start = AI_PTR(weights + 23940);
    dense_8_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_8_bias_array.data = AI_PTR(weights + 25384);
    dense_8_bias_array.data_start = AI_PTR(weights + 25384);
    dense_9_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_9_weights_array.data = AI_PTR(weights + 25460);
    dense_9_weights_array.data_start = AI_PTR(weights + 25460);
    dense_9_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_9_bias_array.data = AI_PTR(weights + 26068);
    dense_9_bias_array.data_start = AI_PTR(weights + 26068);
    dense_10_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_10_weights_array.data = AI_PTR(weights + 26100);
    dense_10_weights_array.data_start = AI_PTR(weights + 26100);
    dense_10_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_10_bias_array.data = AI_PTR(weights + 26132);
    dense_10_bias_array.data_start = AI_PTR(weights + 26132);
  }

  return true;
}


/**  PUBLIC APIs SECTION  *****************************************************/

AI_API_ENTRY
ai_bool ai_predict_house_price_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if ( report && net_ctx )
  {
    ai_network_report r = {
      .model_name        = AI_PREDICT_HOUSE_PRICE_MODEL_NAME,
      .model_signature   = AI_PREDICT_HOUSE_PRICE_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 6771,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .activations       = AI_STRUCT_INIT,
      .params            = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }

  return false;
}

AI_API_ENTRY
ai_error ai_predict_house_price_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_predict_house_price_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_handle ai_predict_house_price_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_predict_house_price_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if ( !net_ctx ) return false;

  ai_bool ok = true;
  ok &= predict_house_price_configure_weights(net_ctx, &params->params);
  ok &= predict_house_price_configure_activations(net_ctx, &params->activations);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_predict_house_price_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_predict_house_price_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_PREDICT_HOUSE_PRICE_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME
