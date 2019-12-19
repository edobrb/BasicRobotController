#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__

#define MAX_STATE_ID_VALUE 10

typedef void(*FunctionPointer)();

class StateMachine { 
public:
  StateMachine(int initialState);
  void transition(int stateId);
  void update();
  void setState(int stateId, FunctionPointer delegate);
  
private:  
  FunctionPointer delegates[MAX_STATE_ID_VALUE];
  int currentState;
};


#endif
