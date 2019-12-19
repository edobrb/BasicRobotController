#include "StateMachine.h"

StateMachine::StateMachine(int initialState) {
  this->currentState = initialState;
}

void StateMachine::transition(int stateId) {
  this->currentState = stateId;
}

void StateMachine::update() {
  this->delegates[this->currentState]();
}

void StateMachine::setState(int stateId, FunctionPointer delegate) {
  this->delegates[stateId] = delegate;
}
