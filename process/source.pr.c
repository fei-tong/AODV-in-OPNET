/* Process model C form file: source.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char source_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 4CCE194A 4CCE194A 1 china-0f9728557 Administrator 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1e80 8                                                                                                                                                                                                                                                                                                                                                                                            ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

#define SEND_STRM 0
//Define node type
#define sink 	1
#define sensor 	2

#define END	        		    	(op_intrpt_type() == OPC_INTRPT_ENDSIM)

//Self-interrupt code and transition condition
#define SEND_DATA_S_CODE		100
#define SEND_DATA_S				((op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code() == SEND_DATA_S_CODE))
//Remote-interrupt code and transition condition
#define SEND_DATA_R_CODE		200
#define SEND_DATA_R				((op_intrpt_type() == OPC_INTRPT_REMOTE) && (op_intrpt_code() == SEND_DATA_R_CODE))

int pk_num;
//function prototype
static void create_and_send_DATA(void);

/* End of Header Block */

#if !defined (VOSD_NO_FIN)
#undef	BIN
#undef	BOUT
#define	BIN		FIN_LOCAL_FIELD(_op_last_line_passed) = __LINE__ - _op_block_origin;
#define	BOUT	BIN
#define	BINIT	FIN_LOCAL_FIELD(_op_last_line_passed) = 0; _op_block_origin = __LINE__;
#else
#define	BINIT
#endif /* #if !defined (VOSD_NO_FIN) */



/* State variable definitions */
typedef struct
	{
	/* Internal state tracking for FSM */
	FSM_SYS_STATE
	/* State Variables */
	Objid	                  		process_id                                      ;
	Objid	                  		node_id                                         ;
	double	                 		start_time                                      ;
	double	                 		interval                                        ;
	Boolean	                		send_DATA                                       ;
	int	                    		node_address                                    ;
	int	                    		traffic_load                                    ;
	Boolean	                		infinity                                        ;
	char	                   		node_type[10]                                   ;
	int	                    		final_dest                                      ;
	} source_state;

#define process_id              		op_sv_ptr->process_id
#define node_id                 		op_sv_ptr->node_id
#define start_time              		op_sv_ptr->start_time
#define interval                		op_sv_ptr->interval
#define send_DATA               		op_sv_ptr->send_DATA
#define node_address            		op_sv_ptr->node_address
#define traffic_load            		op_sv_ptr->traffic_load
#define infinity                		op_sv_ptr->infinity
#define node_type               		op_sv_ptr->node_type
#define final_dest              		op_sv_ptr->final_dest

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	source_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((source_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif

//create DATA pk
static void 
create_and_send_DATA(void)
{
//var
	Packet * pk_DATA;
//in
	FIN(create_and_send_DATA());
//body
	pk_DATA = op_pk_create_fmt("AODV_DATA");
	//op_pk_nfd_set(pk_DATA,"Hop Num",0);
	op_pk_nfd_set(pk_DATA,"Create Time",op_sim_time());
	op_pk_nfd_set(pk_DATA,"SRC",node_address);
	op_pk_nfd_set(pk_DATA,"DEST",final_dest);
	op_pk_nfd_set(pk_DATA,"G",0);
	
	op_pk_send(pk_DATA,SEND_STRM);
//out
	FOUT;
}

/* End of Function Block */

/* Undefine optional tracing in FIN/FOUT/FRET */
/* The FSM has its own tracing code and the other */
/* functions should not have any tracing.		  */
#undef FIN_TRACING
#define FIN_TRACING

#undef FOUTRET_TRACING
#define FOUTRET_TRACING

#if defined (__cplusplus)
extern "C" {
#endif
	void source (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_source_init (int * init_block_ptr);
	void _op_source_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_source_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_source_alloc (VosT_Obtype, int);
	void _op_source_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
source (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (source ());

		{
		/* Temporary Variables */
		//File
		FILE *in;
		char temp_file_name[300];
		/* End of Temporary Variables */


		FSM_ENTER ("source")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "source [init enter execs]")
				FSM_PROFILE_SECTION_IN ("source [init enter execs]", state0_enter_exec)
				{
				// Obtain related ID
				process_id = op_id_self();
				node_id = op_topo_parent(process_id);
				
				op_ima_obj_attr_get(process_id, "Send DATA",&send_DATA);
				op_ima_obj_attr_get(process_id, "Start Time",&start_time);
				op_ima_obj_attr_get(process_id, "Interval",&interval);
				op_ima_obj_attr_get(process_id, "Traffic Load",&traffic_load);
				op_ima_obj_attr_get(process_id, "Final Dest", &final_dest);
				
				op_ima_obj_attr_get(node_id, "MAC Address", &node_address);
				op_ima_obj_attr_get(node_id, "Node Type",node_type);
				
				if(traffic_load<0)
				{
					infinity = OPC_TRUE;
				}else
				{
					infinity = OPC_FALSE;
				}
				
				
				if(send_DATA&&traffic_load<0)
				{
					op_intrpt_schedule_self(op_sim_time() + start_time,SEND_DATA_S_CODE);
				}
				if(send_DATA&&traffic_load>0)
				{
					op_intrpt_schedule_self(op_sim_time() + start_time,SEND_DATA_S_CODE);
					--traffic_load;
				}
				pk_num=0;
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "source [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "init", "idle", "tr_0", "source [init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "source [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"source")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "source [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("source [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (SEND_DATA_S)
			FSM_TEST_COND (SEND_DATA_R)
			FSM_TEST_COND (END)
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 2, state2_enter_exec, ;, "SEND_DATA_S", "", "idle", "send DATA S", "tr_3", "source [idle -> send DATA S : SEND_DATA_S / ]")
				FSM_CASE_TRANSIT (1, 3, state3_enter_exec, ;, "SEND_DATA_R", "", "idle", "send DATA R", "tr_4", "source [idle -> send DATA R : SEND_DATA_R / ]")
				FSM_CASE_TRANSIT (2, 4, state4_enter_exec, ;, "END", "", "idle", "end", "tr_6", "source [idle -> end : END / ]")
				}
				/*---------------------------------------------------------*/



			/** state (send DATA S) enter executives **/
			FSM_STATE_ENTER_FORCED (2, "send DATA S", state2_enter_exec, "source [send DATA S enter execs]")
				FSM_PROFILE_SECTION_IN ("source [send DATA S enter execs]", state2_enter_exec)
				{
				create_and_send_DATA();
				pk_num++;
				if(!infinity && traffic_load>0)
				{
					op_intrpt_schedule_self(op_sim_time() + interval,SEND_DATA_S_CODE);
					--traffic_load;
				}	
				if(infinity)
				{
					op_intrpt_schedule_self(op_sim_time() + interval,SEND_DATA_S_CODE);
				}
				printf("In \"gsrc\",time:%f,\n\
						Have created DATA and sent it to \"gmac\".\n",\
						op_sim_time());
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** state (send DATA S) exit executives **/
			FSM_STATE_EXIT_FORCED (2, "send DATA S", "source [send DATA S exit execs]")


			/** state (send DATA S) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "send DATA S", "idle", "tr_2", "source [send DATA S -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (send DATA R) enter executives **/
			FSM_STATE_ENTER_FORCED (3, "send DATA R", state3_enter_exec, "source [send DATA R enter execs]")
				FSM_PROFILE_SECTION_IN ("source [send DATA R enter execs]", state3_enter_exec)
				{
				create_and_send_DATA();
				pk_num++;
				
				printf("In \"gsrc\",Remote Interruption,time:%f,\n\
						Have created DATA and sent it to \"gmac\".\n",\
						op_sim_time());
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** state (send DATA R) exit executives **/
			FSM_STATE_EXIT_FORCED (3, "send DATA R", "source [send DATA R exit execs]")


			/** state (send DATA R) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "send DATA R", "idle", "tr_5", "source [send DATA R -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (end) enter executives **/
			FSM_STATE_ENTER_UNFORCED (4, "end", state4_enter_exec, "source [end enter execs]")
				FSM_PROFILE_SECTION_IN ("source [end enter execs]", state4_enter_exec)
				{
				if(send_DATA)
				{
					if(op_ima_obj_attr_exists(node_id,"Log File"))
					{	
						op_ima_obj_attr_get(node_id,"Log File",temp_file_name);
						in = fopen(temp_file_name,"at");
						fprintf(in,"节点 %d 共发出数据包个数:%d.(in \"gsrc->end\")\r\n",node_address,pk_num);
						fprintf(in,"发包间隔为：%f seconds.\r\n",interval);
						fprintf(in,"Simulation time: %f s.\r\n",op_sim_time());
						fprintf(in,"End.\r\n\r\n");
						fclose(in);
					}
				}else if(pk_num!=0)
				{
				
					if(op_ima_obj_attr_exists(node_id,"Log File")&&(strcmp(node_type,"sink")==0))
					{	
						op_ima_obj_attr_get(node_id,"Log File",temp_file_name);
						in = fopen(temp_file_name,"at");
						fprintf(in,"节点 %d 通过远程中断，共发出数据包个数:%d.(in \"gsrc->end\")\r\n",node_address,pk_num);
						//fprintf(in,"发包间隔为：%f seconds.\r\n",interval);
						fprintf(in,"Simulation time: %f s.\r\n",op_sim_time());
						fprintf(in,"End.\r\n\r\n");
						fclose(in);
					}
				}
				}
				FSM_PROFILE_SECTION_OUT (state4_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (9,"source")


			/** state (end) exit executives **/
			FSM_STATE_EXIT_UNFORCED (4, "end", "source [end exit execs]")


			/** state (end) transition processing **/
			FSM_TRANSIT_MISSING ("end")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"source")
		}
	}




void
_op_source_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_source_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_source_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_source_svar function. */
#undef process_id
#undef node_id
#undef start_time
#undef interval
#undef send_DATA
#undef node_address
#undef traffic_load
#undef infinity
#undef node_type
#undef final_dest

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_source_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_source_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (source)",
		sizeof (source_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_source_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	source_state * ptr;
	FIN_MT (_op_source_alloc (obtype))

	ptr = (source_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "source [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_source_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	source_state		*prs_ptr;

	FIN_MT (_op_source_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (source_state *)gen_ptr;

	if (strcmp ("process_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->process_id);
		FOUT
		}
	if (strcmp ("node_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_id);
		FOUT
		}
	if (strcmp ("start_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->start_time);
		FOUT
		}
	if (strcmp ("interval" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->interval);
		FOUT
		}
	if (strcmp ("send_DATA" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->send_DATA);
		FOUT
		}
	if (strcmp ("node_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_address);
		FOUT
		}
	if (strcmp ("traffic_load" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->traffic_load);
		FOUT
		}
	if (strcmp ("infinity" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->infinity);
		FOUT
		}
	if (strcmp ("node_type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->node_type);
		FOUT
		}
	if (strcmp ("final_dest" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->final_dest);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

