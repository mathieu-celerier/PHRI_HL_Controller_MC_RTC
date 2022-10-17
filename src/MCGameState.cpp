#include <hl_controller/MCGameState.h>

MCGameState::MCGameState()
{
}

MCGameState::MCGameState(hl_phri_exp::GameState msg) :
status(msg.status), maxPiece(msg.maxPiece), totalTime(msg.totalTime), piecesCaught(msg.piecesCaught), amountCaught(msg.amountCaught), timeLeft(msg.timeLeft),
nextPiece(msg.nextPiece), scaling(msg.scaling), globalOffset(msg.globalOffset), width(msg.width), nbPiecesVisible(msg.nbPiecesVisible)
{
    for(auto piece : msg.pieces) {pieces.push_back(* new MCGamePiece(piece));}
}

MCGameState::~MCGameState()
{
}
