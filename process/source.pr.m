MIL_3_Tfile_Hdr_ 145A 140A modeler 9 4CCE0B8E 4CCE194A 8 china-0f9728557 Administrator 0 0 none none 0 0 none BA11A932 1E05 0 0 0 0 0 0 1e80 8                                                                                                                                                                                                                                                                                                                                                                                  ЋЭg      @  I  M      с  с  х  щ  э  љ  §    е      	Send DATA   џџџџџџџ   џџџџ           џџџџ          џџџџ          џџџџ           ЅZ             
Start Time   џџџџџџџ   џџџџ               џџџџ              џџџџ              џџџџ           ЅZ             Interval   џџџџџџџ   џџџџ               џџџџ              џџџџ              џџџџ      
   10   @$      џџџџ      9   @"      џџџџ      8   @       џџџџ      7   @      џџџџ      6   @      џџџџ      5   @      џџџџ      4   @      џџџџ      3   @      џџџџ      2   @       џџџџ      1   ?№      џџџџ       ЅZ             Traffic Load    џџџџџџџ    џџџџ      џџџџ   infinity(-1)      џџџџџџџџ          џџџџ         infinity(-1)   џџџџџџџџ      0(Remote or No traffic)       џџџџ      1(test)      џџџџ      20      џџџџ      40      (џџџџ      60      <џџџџ      80      Pџџџџ      100      dџџџџ       ЅZ             
Final Dest    џџџџџџџ    џџџџ           џџџџ          џџџџ          џџџџ           ЅZ                 	   begsim intrpt         
   џџџџ   
   doc file            	nd_module      endsim intrpt         
   џџџџ   
   failure intrpts            disabled      intrpt interval         дВI­%У}џџџџ      priority              џџџџ      recovery intrpts            disabled      subqueue                     count    џџџ   
   џџџџ   
      list   	џџџ   
          
      super priority             џџџџ             Objid	\process_id;       Objid	\node_id;       double	\start_time;       double	\interval;       Boolean	\send_DATA;       int	\node_address;       int	\traffic_load;       Boolean	\infinity;       char	\node_type[10];       int	\final_dest;          //File   	FILE *in;   char temp_file_name[300];      #define SEND_STRM 0   //Define node type   #define sink 	1   #define sensor 	2       B#define END	        		    	(op_intrpt_type() == OPC_INTRPT_ENDSIM)       .//Self-interrupt code and transition condition   #define SEND_DATA_S_CODE		100   h#define SEND_DATA_S				((op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code() == SEND_DATA_S_CODE))   0//Remote-interrupt code and transition condition   #define SEND_DATA_R_CODE		200   j#define SEND_DATA_R				((op_intrpt_type() == OPC_INTRPT_REMOTE) && (op_intrpt_code() == SEND_DATA_R_CODE))       int pk_num;   //function prototype   'static void create_and_send_DATA(void);      //create DATA pk   static void    create_and_send_DATA(void)   {   //var   	Packet * pk_DATA;   //in   	FIN(create_and_send_DATA());   //body   )	pk_DATA = op_pk_create_fmt("AODV_DATA");   &	//op_pk_nfd_set(pk_DATA,"Hop Num",0);   4	op_pk_nfd_set(pk_DATA,"Create Time",op_sim_time());   +	op_pk_nfd_set(pk_DATA,"SRC",node_address);   *	op_pk_nfd_set(pk_DATA,"DEST",final_dest);   	op_pk_nfd_set(pk_DATA,"G",0);   	   	op_pk_send(pk_DATA,SEND_STRM);   //out   	FOUT;   }                                                      
   init   
       J       // Obtain related ID   process_id = op_id_self();   &node_id = op_topo_parent(process_id);       8op_ima_obj_attr_get(process_id, "Send DATA",&send_DATA);   :op_ima_obj_attr_get(process_id, "Start Time",&start_time);   6op_ima_obj_attr_get(process_id, "Interval",&interval);   >op_ima_obj_attr_get(process_id, "Traffic Load",&traffic_load);   ;op_ima_obj_attr_get(process_id, "Final Dest", &final_dest);       ;op_ima_obj_attr_get(node_id, "MAC Address", &node_address);   4op_ima_obj_attr_get(node_id, "Node Type",node_type);       if(traffic_load<0)   {   	infinity = OPC_TRUE;   }else   {   	infinity = OPC_FALSE;   }           if(send_DATA&&traffic_load<0)   {   F	op_intrpt_schedule_self(op_sim_time() + start_time,SEND_DATA_S_CODE);   }   if(send_DATA&&traffic_load>0)   {   F	op_intrpt_schedule_self(op_sim_time() + start_time,SEND_DATA_S_CODE);   	--traffic_load;   }   	pk_num=0;   J                     
   џџџџ   
          pr_state        J            
   idle   
                                   
    џџџџ   
          pr_state        J   Z          
   send DATA S   
       
      create_and_send_DATA();   	pk_num++;   if(!infinity && traffic_load>0)   {   D	op_intrpt_schedule_self(op_sim_time() + interval,SEND_DATA_S_CODE);   	--traffic_load;   }	   if(infinity)   {   D	op_intrpt_schedule_self(op_sim_time() + interval,SEND_DATA_S_CODE);   }   printf("In \"gsrc\",time:%f,\n\   1		Have created DATA and sent it to \"gmac\".\n",\   		op_sim_time());   
                     
   џџџџ   
          pr_state        J  Т          
   send DATA R   
       
      create_and_send_DATA();   	pk_num++;       3printf("In \"gsrc\",Remote Interruption,time:%f,\n\   1		Have created DATA and sent it to \"gmac\".\n",\   		op_sim_time());   
                     
   џџџџ   
          pr_state        ў            
   end   
       
      if(send_DATA)   {   /	if(op_ima_obj_attr_exists(node_id,"Log File"))   	{	   9		op_ima_obj_attr_get(node_id,"Log File",temp_file_name);   "		in = fopen(temp_file_name,"at");   W		fprintf(in,"НкЕу %d ЙВЗЂГіЪ§ОнАќИіЪ§:%d.(in \"gsrc->end\")\r\n",node_address,pk_num);   5		fprintf(in,"ЗЂАќМфИєЮЊЃК%f seconds.\r\n",interval);   9		fprintf(in,"Simulation time: %f s.\r\n",op_sim_time());   		fprintf(in,"End.\r\n\r\n");   		fclose(in);   	}   }else if(pk_num!=0)   {       N	if(op_ima_obj_attr_exists(node_id,"Log File")&&(strcmp(node_type,"sink")==0))   	{	   9		op_ima_obj_attr_get(node_id,"Log File",temp_file_name);   "		in = fopen(temp_file_name,"at");   e		fprintf(in,"НкЕу %d ЭЈЙ§дЖГЬжаЖЯЃЌЙВЗЂГіЪ§ОнАќИіЪ§:%d.(in \"gsrc->end\")\r\n",node_address,pk_num);   7		//fprintf(in,"ЗЂАќМфИєЮЊЃК%f seconds.\r\n",interval);   9		fprintf(in,"Simulation time: %f s.\r\n",op_sim_time());   		fprintf(in,"End.\r\n\r\n");   		fclose(in);   	}   }   
                         џџџџ             pr_state                        ј        Ќ    D            
   tr_0   
       џџџџ          џџџџ          
    џџџџ   
          џџџџ                       pr_transition              P   Г     Q   t  Q   ђ          
   tr_2   
       џџџџ          џџџџ          
    џџџџ   
          џџџџ                       pr_transition                 Д     F   ѕ  G   v          
   tr_3   
       
   SEND_DATA_S   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition                i     E  %  D  Ї          
   tr_4   
       
   SEND_DATA_R   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition              T  g     S  Ћ  T  #          
   tr_5   
       џџџџ          џџџџ          
    џџџџ   
          џџџџ                       pr_transition              Ћ       [    њ            
   tr_6   
       
   END   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition                                             