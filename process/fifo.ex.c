///////////////////////////////////////////////////////////////////////////
///////////////////////// LIBRAIRY FILE FOR OPNET /////////////////////////
///////////////////////////////////////////////////////////////////////////
/// 
/// 	Contains:	The FIFO (First In First Out) service
///
///	Company:	National Institute of Standards and Technology
/// 	Written by:	Xavier Pallot
///	Date: 		04/14/00
///
///////////////////////////////////////////////////////////////////////////
/// 	Description:	This library provides the FIFO service for Opnet.
///			The "FIFO" can receive data pointer of any types.
///			A FIFO multiplexing service is also provided by
///			specifying a FIFO number with the data pointer.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
 
#include "fifo.h" 
#include <opnet.h>
///////////////////////////////////////////////////////////////////////////
/////////////////////////// FUNCTION DEFINITION ///////////////////////////
///////////////////////////////////////////////////////////////////////////

/****************************** INIT FIFO ********************************/
/* This function initializes a FIFO structure
/* In:	fifo -> pointer to the FIFO structure to initialize
/*************************************************************************/

void initFifo(sFifo *fifo)
{
fifo->nbrObjects=0;
fifo->firstObject=NULL;
fifo->lastObject=NULL;
}

int getFifoSize(sFifo * fifo)
	{
	return fifo->nbrObjects;
	}
/****************************** NEW FIFO *********************************/
/* This function creates and initializes a new FIFO structure in dynamic
/* memory  
/* Return: a pointer to the new FIFO structure
/*************************************************************************/

sFifo* newFifo()
{
sFifo* fifo;
// allocates the memory for the new fifo
fifo=(sFifo*)op_prg_mem_alloc(sizeof(sFifo));
// initializes this new structure
initFifo(fifo);
return fifo;
}

/*********************** PUT IN FIFO MULTIPLEX  ***************************/
/* This function is used to add a data pointer in a FIFO using the 
/* multeplexing service
/* In :	   fifo -> pointer to the FIFO 
/*	   data -> pointer to the data to add in the FIFO
/*	   fifoNumber -> the number of the FIFO for multiplexing
/* Return: 1 if success, 0 otherwise
/*************************************************************************/

int putInFifoMultiplex(sFifo* fifo,void* data,int fifoNumber)
{
int result;
// adds the data pointer at the end of the fifo
result=putInFifo(fifo,data);
if (result!=0)
	{
	// specify the fifoNumber for multiplexing
	fifo->lastObject->fifoNumber=fifoNumber;
	//printf("*********** EMPIL FIFO %d ID %d ************* ",fifoNumber,fifo);
	//printFifo(*fifo);
	}
return result;
}

/**************************** PUT IN FIFO *******************************/
/* This function is used to add a data pointer at the end of a "FIFO"
/* Note that by default the data pointer is added with a FIFO number equal 
/* to 0 (be careful when you also use the multeplexing service) 
/* In :	   fifo -> pointer to the FIFO 
/*	   data -> pointer to the data to add in the FIFO
/* Return: 1 if success, 0 otherwise
/*************************************************************************/

int putInFifo(sFifo* fifo, void* data)
{
sObject* object;
// allocates dynamic memory for the new fifo object
object=(sObject*)op_prg_mem_alloc(sizeof(sObject));
if (object!=NULL)
	{
	// adds the data in the fifo object
	object->data=data;
	// by default the fifoNumber of the object is 0
	object->fifoNumber=0;
	object->next=NULL;
	// adds the object in the fifo
	if (fifo->nbrObjects<=0)
		{
		// at the beginning if the fifo is empty
		fifo->firstObject=object;
		fifo->lastObject=object;
		}
	else
		{
		// at the end otherwise
		fifo->lastObject->next=object;
		fifo->lastObject=object;
		}
	fifo->nbrObjects++;
	return 1;
	}
else
	{
	return 0;
	}
}

/*********************** GET IN FIFO MULTIPLEX  ***************************/
/* This function is used to get a data pointer from a FIFO using the 
/* multeplexing service
/* In :	   fifo -> pointer to the FIFO 
/*	   fifoNumber -> the number of the FIFO for multiplexing
/* Return: a pointer to the extracted data if success
/*	   NULL otherwise
/*************************************************************************/

void* getInFifoMultiplex(sFifo* fifo,int fifoNumber)
{
sObject* object;
sObject* previousObject;
void *data;
previousObject=NULL;
object=fifo->firstObject;
// scan all the object contained in the fifo
while (object!=NULL)
	{
	// if the object has the wanted fifo number
 	if (object->fifoNumber==fifoNumber)
		{
		// extracts the data pointer
		data=object->data;
		// reorganizes the fifo without the object
		if (previousObject==NULL)
			{
			fifo->firstObject=object->next;
			}
		else
			{
			previousObject->next=object->next;
			if (object==fifo->lastObject)
				{
				fifo->lastObject=previousObject;
				}
			}
		// destroys the object
		op_prg_mem_free(object);
		fifo->nbrObjects--;
		// and returns the data pointer
		//printf("*********** DEPIL FIFO %d ID %d************* ",fifoNumber,fifo);
		//printFifo(*fifo);
		return data;
		}
	// else takes the next object in the fifo
	else
		{
		previousObject=object;
		object=object->next;
		}
	}
return NULL;
}

/*********************** READ IN FIFO MULTIPLEX  ***************************/
/* This function is used to read a data pointer from a FIFO using the 
/* multeplexing service whitout extracting the data
/* In :	   fifo -> pointer to the FIFO 
/*	   fifoNumber -> the number of the FIFO for multiplexing
/* Return: a pointer to the readed data if success
/*	   NULL otherwise
/*************************************************************************/

void* readInFifoMultiplex(sFifo* fifo,int fifoNumber)
{
sObject* object;
sObject* previousObject;
void *data;
previousObject=NULL;
object=fifo->firstObject;
// scan all the object contained in the fifo
while (object!=NULL)
	{
	// if the object has the wanted fifo number
 	if (object->fifoNumber==fifoNumber)
		{
		// extracts the data pointer
		data=object->data;
		// and returns the data pointer
		return data;
		}
	// else takes the next object in the fifo
	else
		{
		previousObject=object;
		object=object->next;
		}
	}
return NULL;
}

/******************** GET FIRST IN FIFO MULTIPLEX ************************/
/* This function is used to get the data pointer which is at the beginning 
/* of a "FIFO" and to return its fifo number
/* In :	   fifo -> pointer to the FIFO
/* Out:    fifoNumber -> the fifo number of the extracted data
/* Return: a pointer to the extracted data if success
/*	   NULL otherwise
/*************************************************************************/

void* getFirstInFifoMultiplex(sFifo *fifo, int *fifoNumber)
{
sObject* object;
void* data;
// if there are some object in the fifo
if (fifo->nbrObjects!=0)
	{
	// extracts the first object from the fifo
	object=fifo->firstObject;
	// extracts the data pointer from this object
	data=object->data;
	// get the fifoNumber corresponding to the data pointer
	*fifoNumber=object->fifoNumber;
	// reorganizes the fifo
	fifo->firstObject=object->next;
	// destroys the extracted object
	op_prg_mem_free(object);
	fifo->nbrObjects--;
	//printf("*********** DEPIL FIRST FIFO %d ID %d ************* ",fifoNumber,fifo);
	//printFifo(*fifo);
	return data;
	}
else
	{
	// returns null if there is nothing in the fifo
	return NULL;
	}
}

/******************** READ FIRST IN FIFO MULTIPLEX ***********************/
/* This function is used to read the data pointer which is at the beginning 
/* of a "FIFO" and to return its fifo number, whithout extracting the data
/* In :	   fifo -> pointer to the FIFO
/* Out:    fifoNumber -> the fifo number of the readed data
/* Return: a pointer to the readed data if success
/*	   NULL otherwise
/*************************************************************************/

void* readFirstInFifoMultiplex(sFifo *fifo, int *fifoNumber)
{
sObject* object;
void* data;
// if there are some object in the fifo
if (fifo->nbrObjects!=0)
	{
	// extracts the first object from the fifo
	object=fifo->firstObject;
	// extracts the data pointer from this object
	data=object->data;
	// get the fifoNumber corresponding to the data pointer
	*fifoNumber=object->fifoNumber;
	return data;
	}
else
	{
	// returns null if there is nothing in the fifo
	return NULL;
	}
}

/**************************** GET IN FIFO *******************************/
/* This function is used to get the data pointer which is at the beginning 
/* of a "FIFO"
/* In :	   fifo -> pointer to the FIFO 
/* Return: a pointer to the extracted data if success
/*	   NULL otherwise
/*************************************************************************/

void* getInFifo(sFifo *fifo)
{
sObject* object;
void *data;
// if there are some object in the fifo
if (fifo->nbrObjects!=0)
	{
	// extracts the first object from the fifo
	object=fifo->firstObject;
	// extracts the data pointer from this object
	data=object->data;
	// reorganizes the fifo
	fifo->firstObject=object->next;
	// destroys the extracted object
	op_prg_mem_free(object);
	fifo->nbrObjects--;
	return data;
	}
else
	{
	// returns null if there is nothing in the fifo
	return NULL;
	}
}

/**************************** READ IN FIFO *******************************/
/* This function is used to read the data pointer which is at the beginning 
/* of a "FIFO" whitout extracting the data from the queue
/* In :	   fifo -> pointer to the FIFO 
/* Return: a pointer to the readed data if success
/*	   NULL otherwise
/*************************************************************************/

void* readInFifo(sFifo *fifo)
{
sObject* object;
void *data;
// if there are some object in the fifo
if (fifo->nbrObjects!=0)
	{
	// extracts the first object from the fifo
	object=fifo->firstObject;
	// extracts the data pointer from this object
	data=object->data;
	return data;
	}
else
	{
	// returns null if there is nothing in the fifo
	return NULL;
	}
}

/**************************** DESTROY FIFO *******************************/
/* This function destroy a dynamic FIFO structure which was created by the 
/* newFifo function. Thus this function also destroys all the objects and 
/* data which are contained in the FIFO.
/* In: fifo -> a pointer to the FIFO structure to destroy
/*************************************************************************/

void destroyFifo(sFifo* fifo)
{
void *data;
// extract all data from the fifo
do 
	{
	data=getInFifo(fifo);
	// destroy the data: work ??
	op_prg_mem_free(data);
	}
while (data!=NULL);
// destroy the fifo
op_prg_mem_free(fifo);
}


/***************************** PRINT FIFO ********************************/
/* This function display a FIFO on the computer screen
/* In :	   fifo -> the FIFO to display
/*************************************************************************/

void printFifo(sFifo fifo)
{
int* i;
int nbr;
sObject* object;
object=fifo.firstObject;
nbr=1;
while(object!=NULL)
	{
	i=(int*)(object->data);
	printf("      /** Element %d:: %d\n",nbr,*i);
	object=object->next;
	nbr++;
	}
if (nbr<2)
	printf("      /** list is empty\n");
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

