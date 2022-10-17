#include <hl_controller/MCGamePiece.h>

MCGamePiece::MCGamePiece()
{
}

MCGamePiece::MCGamePiece(hl_phri_exp::Piece& msg) :
ID(msg.ID), value(msg.value), x(msg.position.x), y(msg.position.y), distToCatcher(msg.distToCatcher), timeToCatcher(msg.timeToCatcher)
{
}

MCGamePiece::~MCGamePiece()
{
}