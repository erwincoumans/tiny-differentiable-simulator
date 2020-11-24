# ifndef CPPAD_CORE_MUL_HPP
# define CPPAD_CORE_MUL_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
AD<Base> operator * (const AD<Base> &left , const AD<Base> &right)
{
    // compute the Base part
    AD<Base> result;
    result.value_  = left.value_ * right.value_;
    CPPAD_ASSERT_UNKNOWN( Parameter(result) );

    // check if there is a recording in progress
    local::ADTape<Base>* tape = AD<Base>::tape_ptr();
    if( tape == CPPAD_NULL )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if left and right tapes match
    bool match_left  = left.tape_id_  == tape_id;
    bool match_right = right.tape_id_ == tape_id;

    // check if left and right are dynamic parameters
    bool dyn_left  = match_left  & (left.ad_type_ == dynamic_enum);
    bool dyn_right = match_right & (right.ad_type_ == dynamic_enum);

    // check if left and right are variables
    bool var_left  = match_left  & (left.ad_type_ != dynamic_enum);
    bool var_right = match_right & (right.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        left.tape_id_ == right.tape_id_ || ! match_left || ! match_right ,
        "Multiply: AD variables or dynamic parameters on different threads."
    );
    if( var_left )
    {   if( var_right )
        {   // result = variable * variable
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::MulvvOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::MulvvOp) == 2 );

            // put operand addresses in tape
            tape->Rec_.PutArg(left.taddr_, right.taddr_);
            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::MulvvOp);
            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
        else if( (! dyn_right) & IdenticalZero(right.value_) )
        {   // result = variable * 0
        }
        else if( (! dyn_right) & IdenticalOne(right.value_) )
        {   // result = variable * 1
            result.make_variable(left.tape_id_, left.taddr_);
        }
        else
        {   // result = variable * parameter
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::MulpvOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::MulpvOp) == 2 );

            // put operand addresses in tape
            addr_t p = right.taddr_;
            if( ! dyn_right )
                p = tape->Rec_.put_con_par(right.value_);
            tape->Rec_.PutArg(p, left.taddr_);
            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::MulpvOp);
            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
    }
    else if( var_right )
    {   if( (! dyn_left) & IdenticalZero(left.value_) )
        {   // result = 0 * variable
        }
        else if( (! dyn_left) & IdenticalOne(left.value_) )
        {   // result = 1 * variable
            result.make_variable(right.tape_id_, right.taddr_);
        }
        else
        {   // result = parameter * variable
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::MulpvOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::MulpvOp) == 2 );

            // put operand addresses in tape
            addr_t p = left.taddr_;
            if( ! dyn_left )
                p = tape->Rec_.put_con_par(left.value_);
            tape->Rec_.PutArg(p, right.taddr_);
            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::MulpvOp);
            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
    }
    else if( dyn_left | dyn_right )
    {   addr_t arg0 = left.taddr_;
        addr_t arg1 = right.taddr_;
        if( ! dyn_left )
            arg0 = tape->Rec_.put_con_par(left.value_);
        if( ! dyn_right )
            arg1 = tape->Rec_.put_con_par(right.value_);
        //
        // parameters with a dynamic parameter result
        result.taddr_   = tape->Rec_.put_dyn_par(
            result.value_, local::mul_dyn,   arg0, arg1
        );
        result.tape_id_ = tape_id;
        result.ad_type_ = dynamic_enum;
    }
    return result;
}

// convert other cases into the case above
CPPAD_FOLD_AD_VALUED_BINARY_OPERATOR(*)

} // END CppAD namespace

# endif
