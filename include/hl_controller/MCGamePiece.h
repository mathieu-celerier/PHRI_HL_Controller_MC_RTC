#include <hl_phri_exp/Piece.h>

class MCGamePiece
{
public:
    short unsigned int ID;
    short unsigned int value;
    float x;
    float y;
    float distToCatcher;
    float timeToCatcher;
public:
    MCGamePiece();
    MCGamePiece(hl_phri_exp::Piece& msg);
    ~MCGamePiece();
};
