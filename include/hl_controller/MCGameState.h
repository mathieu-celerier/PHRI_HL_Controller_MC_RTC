#include <string>
#include <vector>

#include <hl_controller/MCGamePiece.h>
// #include <hl_phri_exp/GameState.h>

class MCGameState
{
private:
    std::string status;
    short unsigned int maxPiece;
    short unsigned int totalTime;
    short unsigned int piecesCaught;
    short unsigned int amountCaught;
    short unsigned int timeLeft;
    MCGamePiece nextPiece;
    float scaling;
    float globalOffset;
    short unsigned int width;
    short unsigned int nbPiecesVisible;
    std::vector<MCGamePiece> pieces;
public:
    MCGameState();
    ~MCGameState();
};