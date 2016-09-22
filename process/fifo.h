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
/// 	Description:	This librairy provides the FIFO service for Opnet.
///			The "FIFO" can receive data pointer of any types.
///			A FIFO multiplexing service is also provided by
///			specifying a FIFO number with the data pointer.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#ifndef FIFO
#define FIFO

///////////////////////////////////////////////////////////////////////////
///////////////////////////// TYPE DEFINITION /////////////////////////////
///////////////////////////////////////////////////////////////////////////

//struct sObject;

// structure definition of the objects which are contained in the "FIFO"
typedef struct tf_object
	{
	// pointer to data of any types
	void* data;
	// FIFO number for multiplexage
	int fifoNumber;
	// pointer to the next Object of the "FIFO"				
	struct tf_object * next;	
	} sObject;

// structure definition of the "FIFO"
typedef struct 
	{
	// number of objects contained in the "FIFO"
	int nbrObjects;
	// pointer to the first object of the "FIFO"
	sObject* firstObject;
	// pointer to the last object of the "FIFO"
	sObject* lastObject;
	} sFifo;

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
//////////////////////////// FUNCTIONS HEADING ////////////////////////////
///////////////////////////////////////////////////////////////////////////

/// to init a fifo
extern void initFifo(sFifo *fifo);

/// to create and init a new dynamic fifo
extern sFifo* newFifo();
extern int getFifoSize(sFifo* fifo);

/// to add a data pointer in a fifo using the multeplexing service
extern int putInFifoMultiplex(sFifo* fifo,void* data,int fifoNumber);

/// to add a data pointer at the end of a fifo
extern int putInFifo(sFifo* fifo, void* data);

/// to get a data pointer from the beginning of the fifo
extern void* getInFifo(sFifo *fifo);

/// to read a data pointer from the beginning of the fifo
extern void* readInFifo(sFifo *fifo);

/// to get a data pointer from a fifo using the multiplexing service
extern void* getInFifoMultiplex(sFifo* fifo,int fifoNumber);

/// to read a data pointer from a fifo using the multiplexing service
extern void* readInFifoMultiplex(sFifo* fifo,int fifoNumber);

/// to get a data pointer and its fifo number from the beginning of a fifo
extern void* getFirstInFifoMultiplex(sFifo* fifo,int *fifoNumber);

/// to read a data pointer and its fifo number from the beginning of a fifo
extern void* readFirstInFifoMultiplex(sFifo* fifo,int *fifoNumber);

/// to print the fifo on the screen
extern void printFifo(sFifo fifo);

/// to destroy a dynamic fifo
extern void destroyFifo(sFifo* fifo);

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#endif
