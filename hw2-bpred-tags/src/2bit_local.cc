/*
 * Copyright (c) 2022-2023 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/pred/2bit_local.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"

namespace gem5
{

namespace branch_prediction
{

LocalBP::LocalBP(const LocalBPParams &params)
    : ConditionalPredictor(params),
      localPredictorSize(params.localPredictorSize),
      localCtrBits(params.localCtrBits),
      localPredictorSets(localPredictorSize / localCtrBits),
      localCtrs(localPredictorSets, SatCounter8(localCtrBits)),
      indexMask(localPredictorSets - 1),
      tagMatching(params.tagMatching),
      localTags(localPredictorSets, 0),
      stats(this)
{
    if (!isPowerOf2(localPredictorSize)) {
        fatal("Invalid local predictor size!\n");
    }

    if (!isPowerOf2(localPredictorSets)) {
        fatal("Invalid number of local predictor sets! Check localCtrBits.\n");
    }

    DPRINTF(Fetch, "index mask: %#x\n", indexMask);

    DPRINTF(Fetch, "local predictor size: %i\n",
            localPredictorSize);

    DPRINTF(Fetch, "local counter bits: %i\n", localCtrBits);

    DPRINTF(Fetch, "instruction shift amount: %i\n",
            instShiftAmt);

    DPRINTF(Fetch, "tag matching: %s\n", tagMatching ? "on" : "off");
}

LocalBP::LocalBPStats::LocalBPStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(tagMatches, statistics::units::Count::get(),
               "Number of predictor updates where the stored tag matched "
               "the PC of the branch being updated"),
      ADD_STAT(tagMismatches, statistics::units::Count::get(),
               "Number of predictor updates where the stored tag belonged "
               "to a different branch, so the counter was reset")
{
}

void LocalBP::branchPlaceholder(ThreadID tid, Addr pc,
                                bool uncond, void * &bpHistory)
{
// Placeholder for a function that only returns history items
}

void
LocalBP::updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target, const StaticInstPtr &inst,
                         void * &bp_history)
{
// Place holder for a function that is called to update predictor history
}


bool
LocalBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    bool taken;
    unsigned local_predictor_idx = getLocalIndex(branch_addr);

    DPRINTF(Fetch, "Looking up index %#x\n",
            local_predictor_idx);

    // With tagging on, a counter whose tag belongs to some other branch
    // tells us nothing about this one, so fall back to the safe default.
    if (tagMatching && localTags[local_predictor_idx] != branch_addr) {
        DPRINTF(Fetch, "Tag %#x does not match PC %#x, predicting not "
                "taken.\n", localTags[local_predictor_idx], branch_addr);
        return false;
    }

    uint8_t counter_val = localCtrs[local_predictor_idx];

    DPRINTF(Fetch, "prediction is %i.\n",
            (int)counter_val);

    taken = getPrediction(counter_val);

    return taken;
}

void
LocalBP::update(ThreadID tid, Addr branch_addr, bool taken, void *&bp_history,
                bool squashed, const StaticInstPtr & inst, Addr target)
{
    assert(bp_history == NULL);
    unsigned local_predictor_idx;

    // No state to restore, and we do not update on the wrong
    // path.
    if (squashed) {
        return;
    }

    // Update the local predictor.
    local_predictor_idx = getLocalIndex(branch_addr);

    DPRINTF(Fetch, "Looking up index %#x\n", local_predictor_idx);

    if (tagMatching) {
        if (localTags[local_predictor_idx] == branch_addr) {
            stats.tagMatches++;
        } else {
            // The counter was trained by a different branch. Claim the
            // entry and start it from the weak state matching this
            // branch's real direction, rather than inheriting the other
            // branch's bias.
            stats.tagMismatches++;
            DPRINTF(Fetch, "Tag %#x replaced by PC %#x.\n",
                    localTags[local_predictor_idx], branch_addr);
            localTags[local_predictor_idx] = branch_addr;
            localCtrs[local_predictor_idx] =
                SatCounter8(localCtrBits, getWeakCounterValue(taken));
            return;
        }
    }

    if (taken) {
        DPRINTF(Fetch, "Branch updated as taken.\n");
        localCtrs[local_predictor_idx]++;
    } else {
        DPRINTF(Fetch, "Branch updated as not taken.\n");
        localCtrs[local_predictor_idx]--;
    }
}

inline
bool
LocalBP::getPrediction(uint8_t &count)
{
    // Get the MSB of the count
    return (count >> (localCtrBits - 1));
}

inline
unsigned
LocalBP::getLocalIndex(Addr &branch_addr)
{
    return (branch_addr >> instShiftAmt) & indexMask;
}

inline
uint8_t
LocalBP::getWeakCounterValue(bool taken) const
{
    // getPrediction() returns the counter's MSB, so for an N-bit counter
    // the values [0, 2^(N-1) - 1] predict not-taken and [2^(N-1), 2^N - 1]
    // predict taken. The two weak states straddle that boundary.
    const uint8_t midpoint = 1 << (localCtrBits - 1);
    return taken ? midpoint : (midpoint - 1);
}


} // namespace branch_prediction
} // namespace gem5
