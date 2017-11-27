#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <lp_solve/lp_lib.h>

TEST(MapSparsification, LpSolveApiTest) {
  lprec* lp;
  // Must be 1 more than number of columns.
  REAL row[1 + 3];

  lp = make_lp(0, 3);
  if (lp == NULL) {
    fprintf(stderr, "Unable to create new LP model\n");
  }

  set_add_rowmode(lp, TRUE);

  row[1] = 1.0;
  // Also zero elements must be provided.
  row[2] = 0.0;
  row[3] = 2.0;
  // Constructs the row: +v_1 +2 v_3 >= 3.
  add_constraint(lp, row, GE, 3.0);

  set_add_rowmode(lp, FALSE);

  int ret = solve(lp);
  EXPECT_EQ(OPTIMAL, ret);

  delete_lp(lp);
}

MAPLAB_UNITTEST_ENTRYPOINT
