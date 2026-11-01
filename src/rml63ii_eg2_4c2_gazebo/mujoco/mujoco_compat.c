/* Compiled with the active MuJoCo wheel's headers and library. */
#include <mujoco/mujoco.h>
#include <math.h>
#include <string.h>

int cabinet_version(void) { return mjVERSION_HEADER; }

void cabinet_copy_data(mjData* destination, const mjModel* model, const mjData* source) {
  mj_copyData(destination, model, source);
}

double cabinet_distance(const mjModel* model, const mjData* data, int first,
                        int second, double limit, double* fromto) {
  int reversed = 0;
  if (model->geom_type[first] > model->geom_type[second]) {
    int temporary = first; first = second; second = temporary; reversed = 1;
  }
  mjfCollision collide = mjCOLLISIONFUNC[model->geom_type[first]][model->geom_type[second]];
  if (!collide) return NAN;
  /* 2.3 libccd MPR is a contact query, not a long-range distance solver.
     Metre-scale margins on millimetre meshes can report false penetration.
     Retain exact primitive queries; convex clearance is conservative/capped
     at 5 mm. Actual overlap is still queried with the unchanged shapes. */
  if (collide == mjCOLLISIONFUNC[mjGEOM_MESH][mjGEOM_MESH] && limit > .005) {
    limit = .005;
  }
  mjContact contacts[mjMAXCONPAIR];
  memset(contacts, 0, sizeof(contacts));
  int count = collide(model, data, contacts, first, second, limit);
  double distance = limit;
  if (fromto) memset(fromto, 0, 6 * sizeof(double));
  for (int i = 0; i < count; ++i) {
    if (contacts[i].dist < distance) {
      distance = contacts[i].dist;
      if (fromto) for (int j = 0; j < 3; ++j) {
        double half = 0.5 * distance * contacts[i].frame[j] * (reversed ? -1 : 1);
        fromto[j] = contacts[i].pos[j] - half;
        fromto[j+3] = contacts[i].pos[j] + half;
      }
    }
  }
  return distance;
}
